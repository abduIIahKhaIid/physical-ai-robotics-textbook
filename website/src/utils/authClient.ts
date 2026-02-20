/**
 * Better-Auth client configuration for the Docusaurus frontend.
 * Uses bearer tokens stored in localStorage for cross-origin auth.
 */

import { createAuthClient } from "better-auth/react";

const BEARER_TOKEN_KEY = "bearer_token";
const ANON_TOKEN_KEY = "robotutor_anon_token";

function getAuthUrl(): string {
  if (typeof window !== "undefined" && window.location.hostname.includes(".app.github.dev")) {
    const host = window.location.hostname;
    const authHost = host.replace(/-\d+\.app\.github\.dev$/, "-4000.app.github.dev");
    return `https://${authHost}`;
  }
  return process.env.BETTER_AUTH_URL || "http://localhost:4000";
}

export const authClient = createAuthClient({
  baseURL: getAuthUrl(),
  fetchOptions: {
    auth: {
      type: "Bearer",
      token: () => localStorage.getItem(BEARER_TOKEN_KEY) || "",
    },
  },
});

/** Store the bearer token in localStorage. */
export function setBearerToken(token: string): void {
  localStorage.setItem(BEARER_TOKEN_KEY, token);
}

/** Clear the bearer token from localStorage. */
export function clearBearerToken(): void {
  localStorage.removeItem(BEARER_TOKEN_KEY);
}

/** Get the current bearer token. */
export function getBearerToken(): string | null {
  return localStorage.getItem(BEARER_TOKEN_KEY);
}

/** Store the anonymous token for later session migration. */
export function setAnonToken(token: string): void {
  sessionStorage.setItem(ANON_TOKEN_KEY, token);
}

/** Get and clear the anonymous token (for migration on login). */
export function consumeAnonToken(): string | null {
  const token = sessionStorage.getItem(ANON_TOKEN_KEY);
  if (token) {
    sessionStorage.removeItem(ANON_TOKEN_KEY);
  }
  return token;
}

/** Get the auth service base URL. */
export { getAuthUrl };
