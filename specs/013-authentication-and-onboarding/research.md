# Research: Authentication & Onboarding Profile

**Feature**: 013-authentication-and-onboarding
**Date**: 2026-02-19
**Sources**: Context7 (better-auth/better-auth, llmstxt/better-auth_llms_txt), FastAPI docs

## Findings

### 1. Better-Auth Database Schema (Auto-Created Tables)

**Decision**: Better-Auth auto-creates 4 tables: `user`, `session`, `account`, `verification`. These use TEXT primary keys by default (not UUIDs). Table/column names use camelCase by default but can be customized.

**Rationale**: Better-Auth manages these tables entirely — no manual migrations needed. The `session.token` field is the bearer token sent by clients. The `account` table stores provider-specific auth data (password hash for email/password, OAuth tokens for social providers).

**Key schema details** (from Context7 — Drizzle ORM snapshots):

**user table**:
- `id` TEXT PK
- `name` TEXT NOT NULL
- `email` TEXT NOT NULL UNIQUE
- `emailVerified` BOOLEAN NOT NULL
- `image` TEXT
- `createdAt` TIMESTAMP NOT NULL DEFAULT now()
- `updatedAt` TIMESTAMP NOT NULL

**session table**:
- `id` TEXT PK
- `userId` TEXT FK → user.id (CASCADE)
- `token` TEXT NOT NULL UNIQUE — **this is the bearer token**
- `expiresAt` TIMESTAMP NOT NULL
- `ipAddress` TEXT
- `userAgent` TEXT
- `createdAt` TIMESTAMP NOT NULL DEFAULT now()
- `updatedAt` TIMESTAMP NOT NULL

**account table**:
- `id` TEXT PK
- `accountId` TEXT NOT NULL (external provider ID, equals userId for credential auth)
- `providerId` TEXT NOT NULL ("credential" for email/password)
- `userId` TEXT FK → user.id (CASCADE)
- `accessToken` TEXT
- `refreshToken` TEXT
- `idToken` TEXT
- `accessTokenExpiresAt` TIMESTAMP
- `refreshTokenExpiresAt` TIMESTAMP
- `scope` TEXT
- `password` TEXT (hashed password for credential auth)
- `createdAt` TIMESTAMP NOT NULL DEFAULT now()
- `updatedAt` TIMESTAMP NOT NULL

**verification table**:
- `id` TEXT PK
- `identifier` TEXT NOT NULL
- `value` TEXT NOT NULL
- `expiresAt` TIMESTAMP NOT NULL
- `createdAt` TIMESTAMP
- `updatedAt` TIMESTAMP

**Alternatives considered**: Custom table names via `modelName` config (e.g., `auth_users` instead of `user`). Decided against to keep default Better-Auth behavior and reduce configuration surface.

**Source**: Context7 — github.com/better-auth/better-auth/blob/canary/packages/cli/test/__snapshots__/auth-schema-pg-uuid.txt

---

### 2. Better-Auth Bearer Plugin — Token Flow

**Decision**: Use the Better-Auth Bearer plugin. Client receives token via `set-auth-token` response header after sign-in, stores in `localStorage`, and sends as `Authorization: Bearer <token>` on all requests.

**Rationale**: Bearer tokens work across origins without cookie complexity. FastAPI can validate tokens by querying the Better-Auth `session` table directly: `SELECT * FROM session WHERE token = $1 AND "expiresAt" > now()`.

**Token lifecycle**:
1. Client calls `POST /api/auth/sign-in/email` → Better-Auth returns session + `set-auth-token` header
2. Client stores token: `localStorage.setItem("bearer_token", token)`
3. Client sends on every request: `Authorization: Bearer <token>`
4. FastAPI reads token → queries `session` table → validates expiry → extracts `userId`

**Server setup**:
```typescript
import { betterAuth } from "better-auth";
import { bearer } from "better-auth/plugins";
export const auth = betterAuth({ plugins: [bearer()] });
```

**Client setup**:
```typescript
import { createAuthClient } from "better-auth/react";
export const authClient = createAuthClient({
  baseURL: "http://localhost:4000",
  fetchOptions: {
    auth: { type: "Bearer", token: () => localStorage.getItem("bearer_token") || "" }
  }
});
```

**Alternatives considered**: Cookie-based sessions (Better-Auth default). Rejected due to cross-origin complexity with GitHub Pages → separate auth service → separate FastAPI service.

**Source**: Context7 — github.com/better-auth/better-auth/blob/canary/docs/content/docs/plugins/bearer.mdx

---

### 3. React Client Integration (createAuthClient)

**Decision**: Use `createAuthClient` from `better-auth/react` with `baseURL` pointing to the Better-Auth microservice. Hooks like `useSession()` provide reactive auth state.

**Rationale**: Better-Auth's React integration provides `useSession`, `signIn`, `signUp`, `signOut` hooks. The `baseURL` config handles cross-origin by pointing to the auth service URL.

**Configuration**:
```typescript
// website/src/utils/authClient.ts
import { createAuthClient } from "better-auth/react";
export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_URL || "http://localhost:4000",
  fetchOptions: {
    auth: { type: "Bearer", token: () => localStorage.getItem("bearer_token") || "" }
  }
});
```

**Alternatives considered**: Manual fetch calls to Better-Auth REST endpoints. Rejected because `createAuthClient` provides type safety and session reactivity.

**Source**: Context7 — better-auth.com/llms.txt/docs/installation

---

### 4. Additional User Fields (Onboarding Profile Storage)

**Decision**: Store onboarding profile in a **separate `user_profiles` table** rather than extending Better-Auth's `user` table with `additionalFields`.

**Rationale**: While Better-Auth supports `additionalFields` on the user model, a separate table is better because:
1. Profile data is application-specific, not auth-specific
2. Avoids coupling profile schema changes to Better-Auth migrations
3. Allows independent CRUD operations on profile without touching auth state
4. FastAPI manages the profile table directly via asyncpg (no Node.js dependency for profile operations)

Better-Auth's `additionalFields` approach:
```typescript
// Could do this, but choosing not to:
export const auth = betterAuth({
  user: {
    additionalFields: {
      softwareLevel: { type: "string", defaultValue: "beginner" },
    }
  }
});
```

**Alternatives considered**: `additionalFields` on Better-Auth user table. Rejected due to coupling concerns and the fact that FastAPI needs to manage these fields independently.

**Source**: Context7 — better-auth.com/llms.txt/docs/concepts/database

---

### 5. Trusted Origins / CORS Configuration

**Decision**: Configure `trustedOrigins` in Better-Auth server config to allow the Docusaurus frontend origin. FastAPI's existing CORS middleware will be updated to also allow the auth service origin.

**Rationale**: Better-Auth includes built-in CSRF protection that requires trusted origins. The frontend (GitHub Pages) must be listed as a trusted origin for auth requests to succeed.

**Configuration**:
```typescript
export const auth = betterAuth({
  trustedOrigins: [
    process.env.FRONTEND_URL || "http://localhost:3000",
  ],
  // ...
});
```

FastAPI CORS update:
```python
# Add Better-Auth service origin to allowed origins
cors_origins = ["https://abdullahkhalid.com", "http://localhost:4000"]
```

**Alternatives considered**: Proxy all auth requests through FastAPI to avoid CORS. Rejected due to added complexity and latency.

**Source**: Context7 — context7.com/better-auth/better-auth/llms.txt (server configuration example)

---

### 6. FastAPI Session Validation Strategy

**Decision**: FastAPI validates Bearer tokens by directly querying the Better-Auth `session` table in the shared Neon database. No HTTP call to Better-Auth needed.

**Rationale**: Both services share the same Neon database. A simple SQL query (`SELECT "userId" FROM session WHERE token = $1 AND "expiresAt" > now()`) is faster and more reliable than an HTTP introspection call. This avoids a runtime dependency between FastAPI and the auth service.

**Key implementation detail**: Better-Auth uses camelCase column names (`expiresAt`, `userId`). PostgreSQL requires double-quoting these: `"expiresAt"`, `"userId"`.

**Alternatives considered**: HTTP token introspection (FastAPI calls Better-Auth's `/api/auth/get-session`). Rejected due to added latency, network dependency, and single point of failure.

**Source**: Derived from Better-Auth schema analysis + architecture decision
