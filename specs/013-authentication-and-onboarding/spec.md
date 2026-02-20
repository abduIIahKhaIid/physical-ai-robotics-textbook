# Feature Specification: Authentication & Onboarding Profile

**Feature Branch**: `013-authentication-and-onboarding`
**Created**: 2026-02-19
**Status**: Draft
**Input**: User description: "Add authentication and onboarding questionnaire (software/hardware background), store profile in Neon"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registers an Account (Priority: P1)

A first-time visitor to the Physical AI & Humanoid Robotics textbook wants to create an account so their chat history, preferences, and progress are saved across sessions. They provide an email and password, and the system creates their account and logs them in immediately.

**Why this priority**: Without account creation, no other personalized features (onboarding, profile storage, persistent history) can function. This is the foundational building block.

**Independent Test**: Can be fully tested by submitting a registration form with email and password, verifying the account is created, and confirming the user receives a valid session token that grants access to authenticated endpoints.

**Acceptance Scenarios**:

1. **Given** a visitor with no account, **When** they submit a valid email and password via the registration form, **Then** the system creates their account, returns a valid authentication token, and the user is immediately logged in.
2. **Given** a visitor attempts to register, **When** they provide an email that is already registered, **Then** the system returns a clear error message indicating the email is taken, without revealing whether the account exists (generic "registration failed" message for security).
3. **Given** a visitor submits a registration form, **When** the password does not meet minimum strength requirements (at least 8 characters), **Then** the system rejects the registration with a specific validation error.
4. **Given** a visitor submits a registration form, **When** the email format is invalid, **Then** the system rejects the registration with a validation error.

---

### User Story 2 - Returning User Logs In (Priority: P1)

A returning user wants to log in with their email and password so they can access their saved chat sessions, onboarding profile, and personalized experience.

**Why this priority**: Login is co-equal with registration as the core authentication gate. Without login, registered users cannot return to their personalized experience.

**Independent Test**: Can be fully tested by logging in with valid credentials, verifying a token is returned, and confirming the token grants access to the user's existing sessions and profile.

**Acceptance Scenarios**:

1. **Given** a registered user, **When** they submit correct email and password, **Then** the system returns a valid authentication token and the user can access their profile and sessions.
2. **Given** a registered user, **When** they submit an incorrect password, **Then** the system returns a generic "invalid credentials" error without revealing whether the email exists.
3. **Given** a user with a valid token, **When** the token expires, **Then** the system returns a 401 error and the user must re-authenticate.
4. **Given** an anonymous user with existing chat sessions, **When** they register or log in, **Then** their anonymous sessions are migrated to their new authenticated account.

---

### User Story 3 - New User Completes Onboarding Questionnaire (Priority: P2)

After registering, a new user is presented with a short onboarding questionnaire that captures their background in software development, hardware/robotics experience, and preferred learning pace. This profile is stored and used to personalize their textbook experience.

**Why this priority**: The onboarding questionnaire is the primary mechanism for collecting the user profile data that drives content personalization. It depends on authentication (P1) being in place first.

**Independent Test**: Can be fully tested by completing the questionnaire after registration, verifying answers are stored in the database, and confirming the profile can be retrieved via the profile endpoint.

**Acceptance Scenarios**:

1. **Given** a newly registered user who has not completed onboarding, **When** they access the textbook, **Then** they are prompted to complete the onboarding questionnaire before proceeding.
2. **Given** a user viewing the onboarding questionnaire, **When** they answer all required questions and submit, **Then** their profile is saved and they are directed to the textbook content.
3. **Given** a user viewing the onboarding questionnaire, **When** they choose to skip the questionnaire, **Then** their profile is saved with default values (beginner level, no hardware specified) and they proceed to the textbook.
4. **Given** a user who previously completed onboarding, **When** they log in again, **Then** they are not prompted to repeat the questionnaire and proceed directly to the textbook.

---

### User Story 4 - User Updates Their Profile (Priority: P3)

An authenticated user wants to update their onboarding profile answers (e.g., they acquired new hardware, improved their skills) so their personalized experience reflects their current state.

**Why this priority**: Profile updates are important for long-term engagement but not required for initial launch. Users can always re-do the questionnaire later.

**Independent Test**: Can be fully tested by accessing the profile settings, modifying answers, saving, and verifying the updated profile is returned on subsequent requests.

**Acceptance Scenarios**:

1. **Given** an authenticated user with a saved profile, **When** they access their profile settings, **Then** they see their current onboarding answers pre-filled.
2. **Given** an authenticated user editing their profile, **When** they change one or more answers and save, **Then** the updated profile is persisted and a confirmation is shown.
3. **Given** an authenticated user, **When** they change their experience level from "beginner" to "intermediate", **Then** subsequent personalized content reflects the updated level.

---

### User Story 5 - User Logs Out (Priority: P3)

An authenticated user wants to log out to end their session, especially on shared or public devices.

**Why this priority**: Logout is a basic security feature but lower priority since tokens expire naturally.

**Independent Test**: Can be fully tested by logging out and verifying the token no longer grants access to authenticated endpoints.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they log out, **Then** their current token is invalidated and subsequent requests with that token return 401.
2. **Given** a user who has logged out, **When** they attempt to access authenticated endpoints, **Then** they are redirected to the login flow.

---

### Edge Cases

- What happens when a user registers with the same email using different letter casing (e.g., "User@Email.com" vs "user@email.com")? Email comparison must be case-insensitive.
- How does the system handle concurrent registration attempts with the same email? Only the first attempt succeeds; the second receives a registration error.
- What happens when the database is temporarily unavailable during registration or login? The system returns a 503 Service Unavailable error with a user-friendly message.
- What happens when a user's token is used from a different IP address? Tokens remain valid regardless of IP to support mobile/network switching.
- What happens when the onboarding questionnaire schema is updated after a user has already completed it? Existing profiles retain their data; new fields use default values until the user updates their profile.
- How does the system handle extremely long input in questionnaire free-text fields? Input is truncated or rejected with a validation error if it exceeds maximum length (500 characters).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow new users to create an account using email and password.
- **FR-002**: System MUST validate email format and enforce minimum password strength (8+ characters) during registration.
- **FR-003**: System MUST store passwords securely using Better-Auth's built-in password hashing (bcrypt by default, configurable).
- **FR-004**: System MUST authenticate returning users via email and password, returning a time-limited bearer token (via Better-Auth Bearer plugin). The client stores the token in localStorage and sends it as `Authorization: Bearer <token>` to all backend services.
- **FR-005**: System MUST treat email addresses as case-insensitive for all lookups and uniqueness checks.
- **FR-006**: System MUST support token expiration with a configurable lifetime (default: 24 hours).
- **FR-007**: System MUST provide a logout mechanism that invalidates the user's current token.
- **FR-008**: System MUST migrate existing anonymous chat sessions to the authenticated user's account upon first registration or login.
- **FR-009**: System MUST present an onboarding questionnaire to newly registered users who have not yet completed it.
- **FR-010**: System MUST collect the following profile information via the onboarding questionnaire:
  - **Software experience level**: beginner, intermediate, advanced
  - **Primary programming language(s)**: free-text (max 200 characters)
  - **Hardware/robotics experience level**: none, hobbyist, academic, professional
  - **Available hardware**: multi-select from predefined options (e.g., "Jetson Nano/Orin", "Raspberry Pi", "ROS 2 workstation", "GPU workstation", "None/simulation only")
  - **Learning goal**: free-text (max 500 characters)
  - **Preferred pace**: self-paced, structured weekly
- **FR-011**: System MUST allow users to skip the onboarding questionnaire, storing default profile values.
- **FR-012**: System MUST allow authenticated users to view and update their onboarding profile at any time.
- **FR-013**: System MUST persist all user profile data in the existing Neon Postgres database.
- **FR-014**: System MUST maintain backward compatibility with the existing anonymous user flow — unauthenticated users can still use the chatbot with limited features.
- **FR-015**: System MUST return consistent, non-leaking error messages for authentication failures (not revealing whether an email is registered).

### Key Entities

- **User Account**: Represents a registered user. Key attributes: unique email, hashed password, account status (active/disabled), creation timestamp, last login timestamp. Extends the existing `users` table concept from anonymous tokens to credential-based accounts.
- **Onboarding Profile**: Represents a user's background and preferences. Key attributes: software experience level, programming languages, hardware experience level, available hardware list, learning goal, preferred pace, onboarding completion status, last updated timestamp. One-to-one relationship with User Account.
- **Authentication Token / Session**: Represents an active session managed by Better-Auth (separate Node.js microservice). Key attributes: token value, associated user, expiration timestamp, creation timestamp. Better-Auth manages its own session table in the shared Neon database; FastAPI validates sessions by querying the shared database or via token introspection.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete registration (email + password) in under 30 seconds.
- **SC-002**: Users can log in and reach the textbook content in under 10 seconds.
- **SC-003**: 90% of new users complete the onboarding questionnaire on their first session (not skipping).
- **SC-004**: Users can complete the onboarding questionnaire in under 2 minutes.
- **SC-005**: System supports at least 500 concurrent authenticated users without performance degradation.
- **SC-006**: All authentication error responses use generic messages that do not leak account existence information.
- **SC-007**: Anonymous-to-authenticated session migration succeeds for 100% of users with existing anonymous sessions.
- **SC-008**: Profile data is retrievable within 1 second of being saved.

## Clarifications

### Session 2026-02-19

- Q: Where does Better-Auth live in this architecture (Docusaurus client + FastAPI backend)? → A: Better-Auth runs as a separate Node.js microservice alongside FastAPI. Both services connect to the same Neon Postgres database. The Docusaurus static site calls Better-Auth for auth operations (signup/signin/session) and FastAPI for chat/RAG operations. FastAPI validates Better-Auth sessions via shared database or token introspection.
- Q: What session strategy — cookie-based or bearer token? → A: Bearer tokens via Better-Auth's Bearer plugin. Client stores token in localStorage after sign-in, sends via `Authorization: Bearer <token>` header to both Better-Auth and FastAPI. Cross-origin safe without SameSite cookie complexity. FastAPI validates by querying the Better-Auth session table in the shared Neon database.
- Q: Does ChatKit require login or support guest access? → A: Deferred (user advanced to /sp.plan). Assumed: Guest access with limits per FR-014 and existing rate-limiting (10 msg/min anonymous, 20 msg/min authenticated).

## Assumptions

- The existing Neon Postgres database and asyncpg connection pool (from spec 010) will be extended, not replaced.
- The existing migration system (`backend/db/migrate.py`) will be used for schema changes.
- Better-Auth manages its own database tables (user, session, account, verification) in the shared Neon database. The existing `users` table from spec 010 will be migrated/linked to Better-Auth's user table, not altered in place.
- Password hashing is handled by Better-Auth (bcrypt by default); no custom hashing implementation needed.
- Token lifetime of 24 hours is appropriate for an educational textbook application (not a banking app).
- The onboarding questionnaire questions are fixed at launch; dynamic/admin-editable questions are out of scope.
- The ChatKit UI widget (spec 011) will be updated to show login/register controls, but the UI changes are out of scope for this spec — this spec covers the backend API only.
- Rate limiting (already implemented) will apply to auth endpoints to prevent brute-force attacks.
