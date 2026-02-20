# Auth Service API Contract (Better-Auth Node.js Microservice)

**Service**: `auth-service/` (new Node.js project)
**Base URL**: `BETTER_AUTH_URL` (e.g., `https://auth.example.com` or `http://localhost:3000`)
**Protocol**: HTTPS (production), HTTP (development)

> These endpoints are provided by Better-Auth out of the box. Listed here for integration reference.

## Better-Auth Built-in Endpoints

### POST /api/auth/sign-up/email

Register a new user with email and password.

**Request**:
```json
{
  "name": "string",
  "email": "string",
  "password": "string (min 8 chars)"
}
```

**Response (200)**:
```json
{
  "user": {
    "id": "string",
    "name": "string",
    "email": "string",
    "emailVerified": false,
    "image": null,
    "createdAt": "ISO8601",
    "updatedAt": "ISO8601"
  },
  "session": {
    "id": "string",
    "userId": "string",
    "token": "string (bearer token)",
    "expiresAt": "ISO8601"
  }
}
```

**Response Headers** (Bearer plugin):
```
set-auth-token: <bearer_token>
```

**Errors**:
- 400: Invalid email/password format
- 422: Email already registered (generic message for security)

---

### POST /api/auth/sign-in/email

Authenticate an existing user.

**Request**:
```json
{
  "email": "string",
  "password": "string"
}
```

**Response (200)**: Same shape as sign-up response.

**Response Headers** (Bearer plugin):
```
set-auth-token: <bearer_token>
```

**Errors**:
- 401: Invalid credentials (generic message)

---

### POST /api/auth/sign-out

Sign out the current user (invalidate session).

**Headers**:
```
Authorization: Bearer <token>
```

**Response (200)**:
```json
{
  "success": true
}
```

---

### GET /api/auth/get-session

Retrieve the current session and user info.

**Headers**:
```
Authorization: Bearer <token>
```

**Response (200)**:
```json
{
  "session": {
    "id": "string",
    "userId": "string",
    "token": "string",
    "expiresAt": "ISO8601"
  },
  "user": {
    "id": "string",
    "name": "string",
    "email": "string",
    "emailVerified": false,
    "image": null
  }
}
```

**Response (401)**: No valid session.

---

## Better-Auth Server Configuration

```typescript
// auth-service/src/auth.ts
import { betterAuth } from "better-auth";
import { bearer } from "better-auth/plugins";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,  // Same Neon DB
  }),
  baseURL: process.env.BETTER_AUTH_URL,
  secret: process.env.BETTER_AUTH_SECRET,  // min 32 chars
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true,
  },
  session: {
    expiresIn: 60 * 60 * 24,  // 24 hours (per spec FR-006)
    updateAge: 60 * 60,        // Refresh hourly
  },
  trustedOrigins: [
    process.env.FRONTEND_URL,  // GitHub Pages URL
  ],
  plugins: [bearer()],
});
```

## Better-Auth Client Configuration

```typescript
// website/src/utils/authClient.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000",
  fetchOptions: {
    auth: {
      type: "Bearer",
      token: () => localStorage.getItem("bearer_token") || "",
    },
  },
});
```
