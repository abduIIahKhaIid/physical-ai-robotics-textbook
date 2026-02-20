/**
 * LoginForm — Email/password sign-in form.
 */

import React, { useState, FormEvent } from "react";
import { useAuth } from "../AuthProvider";

interface LoginFormProps {
  onSuccess?: () => void;
  onSwitchToRegister?: () => void;
}

export function LoginForm({ onSuccess, onSwitchToRegister }: LoginFormProps) {
  const { signIn } = useAuth();
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError("");
    setIsSubmitting(true);

    const result = await signIn(email, password);
    setIsSubmitting(false);

    if (result.error) {
      setError(result.error);
    } else {
      onSuccess?.();
    }
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form">
      <h3 className="auth-form-title">Sign In</h3>

      {error && <div className="auth-form-error">{error}</div>}

      <label className="auth-form-label">
        Email
        <input
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
          className="auth-form-input"
          placeholder="you@example.com"
          autoComplete="email"
        />
      </label>

      <label className="auth-form-label">
        Password
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
          className="auth-form-input"
          placeholder="Your password"
          autoComplete="current-password"
        />
      </label>

      <button type="submit" disabled={isSubmitting} className="auth-form-button">
        {isSubmitting ? "Signing in..." : "Sign In"}
      </button>

      {onSwitchToRegister && (
        <p className="auth-form-switch">
          Don't have an account?{" "}
          <button type="button" onClick={onSwitchToRegister} className="auth-form-link">
            Sign up
          </button>
        </p>
      )}
    </form>
  );
}
