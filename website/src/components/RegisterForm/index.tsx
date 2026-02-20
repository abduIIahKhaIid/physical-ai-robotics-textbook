/**
 * RegisterForm — Email/password registration form.
 */

import React, { useState, FormEvent } from "react";
import { useAuth } from "../AuthProvider";

interface RegisterFormProps {
  onSuccess?: () => void;
  onSwitchToLogin?: () => void;
}

export function RegisterForm({ onSuccess, onSwitchToLogin }: RegisterFormProps) {
  const { signUp } = useAuth();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError("");
    setIsSubmitting(true);

    const result = await signUp(name, email, password);
    setIsSubmitting(false);

    if (result.error) {
      setError(result.error);
    } else {
      onSuccess?.();
    }
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form">
      <h3 className="auth-form-title">Create Account</h3>

      {error && <div className="auth-form-error">{error}</div>}

      <label className="auth-form-label">
        Name
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
          className="auth-form-input"
          placeholder="Your name"
          autoComplete="name"
        />
      </label>

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
          minLength={8}
          className="auth-form-input"
          placeholder="Min 8 characters"
          autoComplete="new-password"
        />
      </label>

      <button type="submit" disabled={isSubmitting} className="auth-form-button">
        {isSubmitting ? "Creating account..." : "Sign Up"}
      </button>

      {onSwitchToLogin && (
        <p className="auth-form-switch">
          Already have an account?{" "}
          <button type="button" onClick={onSwitchToLogin} className="auth-form-link">
            Sign in
          </button>
        </p>
      )}
    </form>
  );
}
