/**
 * AuthProvider — React context for authentication state.
 * Wraps the app and exposes auth state + actions to all children.
 */

import React, { createContext, useContext, useState, useCallback, useEffect } from "react";
import {
  authClient,
  setBearerToken,
  clearBearerToken,
  getBearerToken,
  consumeAnonToken,
} from "../../utils/authClient";
import { getChatApiUrl } from "../../utils/chatConfig";

interface AuthUser {
  id: string;
  email: string;
  name: string;
}

interface AuthContextType {
  user: AuthUser | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  onboardingCompleted: boolean | null;
  signUp: (name: string, email: string, password: string) => Promise<{ error?: string }>;
  signIn: (email: string, password: string) => Promise<{ error?: string }>;
  signOut: () => Promise<void>;
  checkOnboarding: () => Promise<void>;
  setOnboardingCompleted: (value: boolean) => void;
}

const AuthContext = createContext<AuthContextType | null>(null);

export function useAuth(): AuthContextType {
  const ctx = useContext(AuthContext);
  if (!ctx) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return ctx;
}

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<AuthUser | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [onboardingCompleted, setOnboardingCompleted] = useState<boolean | null>(null);

  const isAuthenticated = user !== null;

  // Migrate anonymous sessions after authentication
  const migrateAnonymousSessions = useCallback(async (token: string) => {
    const anonToken = consumeAnonToken();
    if (!anonToken) return;

    try {
      const backendUrl = getChatApiUrl();
      await fetch(`${backendUrl}/api/v1/auth/migrate-sessions`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({ anonymous_token: anonToken }),
      });
    } catch {
      // Non-critical — silent fail
    }
  }, []);

  // Check onboarding status
  const checkOnboarding = useCallback(async () => {
    const token = getBearerToken();
    if (!token) return;

    try {
      const backendUrl = getChatApiUrl();
      const res = await fetch(`${backendUrl}/api/v1/profile`, {
        headers: { Authorization: `Bearer ${token}` },
      });
      if (res.ok) {
        const data = await res.json();
        setOnboardingCompleted(data.onboarding_completed);
      }
    } catch {
      // Non-critical
    }
  }, []);

  // Check session on mount
  useEffect(() => {
    const token = getBearerToken();
    if (!token) {
      setIsLoading(false);
      return;
    }

    // Validate token by fetching session from Better-Auth
    authClient.getSession().then((result) => {
      if (result?.data?.user) {
        const u = result.data.user;
        setUser({ id: u.id, email: u.email, name: u.name });
        checkOnboarding();
      } else {
        // Token invalid — clear it
        clearBearerToken();
      }
      setIsLoading(false);
    }).catch(() => {
      clearBearerToken();
      setIsLoading(false);
    });
  }, [checkOnboarding]);

  const signUp = useCallback(async (name: string, email: string, password: string) => {
    try {
      const result = await authClient.signUp.email({
        name,
        email,
        password,
        fetchOptions: {
          onSuccess: (ctx) => {
            // Extract bearer token from response header
            const token = ctx.response.headers.get("set-auth-token");
            if (token) {
              setBearerToken(token);
              migrateAnonymousSessions(token);
            }
          },
        },
      });

      if (result?.data?.user) {
        const u = result.data.user;
        setUser({ id: u.id, email: u.email, name: u.name });
        setOnboardingCompleted(false);
        return {};
      }
      return { error: "Registration failed. Please try again." };
    } catch {
      return { error: "Registration failed. Please try again." };
    }
  }, [migrateAnonymousSessions]);

  const signIn = useCallback(async (email: string, password: string) => {
    try {
      const result = await authClient.signIn.email({
        email,
        password,
        fetchOptions: {
          onSuccess: (ctx) => {
            const token = ctx.response.headers.get("set-auth-token");
            if (token) {
              setBearerToken(token);
              migrateAnonymousSessions(token);
            }
          },
        },
      });

      if (result?.data?.user) {
        const u = result.data.user;
        setUser({ id: u.id, email: u.email, name: u.name });
        await checkOnboarding();
        return {};
      }
      return { error: "Invalid email or password." };
    } catch {
      return { error: "Invalid email or password." };
    }
  }, [migrateAnonymousSessions, checkOnboarding]);

  const signOut = useCallback(async () => {
    try {
      await authClient.signOut();
    } catch {
      // Continue with local cleanup even if remote signout fails
    }
    clearBearerToken();
    setUser(null);
    setOnboardingCompleted(null);
  }, []);

  return (
    <AuthContext.Provider
      value={{
        user,
        isAuthenticated,
        isLoading,
        onboardingCompleted,
        signUp,
        signIn,
        signOut,
        checkOnboarding,
        setOnboardingCompleted,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}
