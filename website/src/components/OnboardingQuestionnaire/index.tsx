/**
 * OnboardingQuestionnaire — Collects software/hardware background after registration.
 * 6 fields per FR-010: software_level, programming_languages, hardware_level,
 * available_hardware, learning_goal, preferred_pace.
 */

import React, { useState, FormEvent, useEffect } from "react";
import { useAuth } from "../AuthProvider";
import { getBearerToken } from "../../utils/authClient";
import { getChatApiUrl } from "../../utils/chatConfig";

const HARDWARE_OPTIONS = [
  { value: "jetson_nano_orin", label: "Jetson Nano / Orin" },
  { value: "raspberry_pi", label: "Raspberry Pi" },
  { value: "ros2_workstation", label: "ROS 2 Workstation" },
  { value: "gpu_workstation", label: "GPU Workstation" },
  { value: "simulation_only", label: "Simulation Only" },
] as const;

interface OnboardingQuestionnaireProps {
  /** Pre-fill with existing profile data (for editing). */
  initialData?: {
    software_level?: string;
    programming_languages?: string;
    hardware_level?: string;
    available_hardware?: string[];
    learning_goal?: string;
    preferred_pace?: string;
  };
  onComplete?: () => void;
  /** If true, show as PATCH update instead of initial onboarding. */
  isEdit?: boolean;
}

export function OnboardingQuestionnaire({
  initialData,
  onComplete,
  isEdit = false,
}: OnboardingQuestionnaireProps) {
  const { setOnboardingCompleted } = useAuth();

  const [softwareLevel, setSoftwareLevel] = useState(initialData?.software_level || "beginner");
  const [programmingLanguages, setProgrammingLanguages] = useState(initialData?.programming_languages || "");
  const [hardwareLevel, setHardwareLevel] = useState(initialData?.hardware_level || "none");
  const [availableHardware, setAvailableHardware] = useState<string[]>(initialData?.available_hardware || []);
  const [learningGoal, setLearningGoal] = useState(initialData?.learning_goal || "");
  const [preferredPace, setPreferredPace] = useState(initialData?.preferred_pace || "self_paced");
  const [error, setError] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  const toggleHardware = (value: string) => {
    setAvailableHardware((prev) =>
      prev.includes(value) ? prev.filter((h) => h !== value) : [...prev, value]
    );
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError("");
    setIsSubmitting(true);

    const token = getBearerToken();
    if (!token) {
      setError("You must be signed in to complete onboarding.");
      setIsSubmitting(false);
      return;
    }

    const backendUrl = getChatApiUrl();
    const method = isEdit ? "PATCH" : "POST";
    const body = {
      software_level: softwareLevel,
      programming_languages: programmingLanguages,
      hardware_level: hardwareLevel,
      available_hardware: availableHardware,
      learning_goal: learningGoal,
      preferred_pace: preferredPace,
    };

    try {
      const res = await fetch(`${backendUrl}/api/v1/profile`, {
        method,
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify(body),
      });

      if (!res.ok) {
        const errData = await res.json().catch(() => null);
        setError(errData?.error?.message || "Failed to save profile. Please try again.");
        setIsSubmitting(false);
        return;
      }

      setOnboardingCompleted(true);
      onComplete?.();
    } catch {
      setError("Network error. Please try again.");
    }

    setIsSubmitting(false);
  };

  const handleSkip = async () => {
    // Save defaults
    setIsSubmitting(true);
    const token = getBearerToken();
    if (!token) {
      setOnboardingCompleted(true);
      onComplete?.();
      return;
    }

    const backendUrl = getChatApiUrl();
    try {
      await fetch(`${backendUrl}/api/v1/profile`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          software_level: "beginner",
          hardware_level: "none",
          preferred_pace: "self_paced",
        }),
      });
    } catch {
      // Non-critical
    }

    setOnboardingCompleted(true);
    onComplete?.();
    setIsSubmitting(false);
  };

  return (
    <form onSubmit={handleSubmit} className="onboarding-form">
      <h3 className="auth-form-title">
        {isEdit ? "Edit Profile" : "Tell us about yourself"}
      </h3>
      <p className="onboarding-subtitle">
        {isEdit
          ? "Update your learning preferences."
          : "Help us personalize your learning experience."}
      </p>

      {error && <div className="auth-form-error">{error}</div>}

      {/* Software Level */}
      <fieldset className="onboarding-fieldset">
        <legend>Software Experience</legend>
        {(["beginner", "intermediate", "advanced"] as const).map((level) => (
          <label key={level} className="onboarding-radio">
            <input
              type="radio"
              name="software_level"
              value={level}
              checked={softwareLevel === level}
              onChange={() => setSoftwareLevel(level)}
            />
            {level.charAt(0).toUpperCase() + level.slice(1)}
          </label>
        ))}
      </fieldset>

      {/* Programming Languages */}
      <label className="auth-form-label">
        Programming Languages
        <input
          type="text"
          value={programmingLanguages}
          onChange={(e) => setProgrammingLanguages(e.target.value.slice(0, 200))}
          className="auth-form-input"
          placeholder="e.g. Python, C++, ROS 2"
          maxLength={200}
        />
      </label>

      {/* Hardware Level */}
      <fieldset className="onboarding-fieldset">
        <legend>Hardware Experience</legend>
        {(["none", "hobbyist", "academic", "professional"] as const).map((level) => (
          <label key={level} className="onboarding-radio">
            <input
              type="radio"
              name="hardware_level"
              value={level}
              checked={hardwareLevel === level}
              onChange={() => setHardwareLevel(level)}
            />
            {level.charAt(0).toUpperCase() + level.slice(1)}
          </label>
        ))}
      </fieldset>

      {/* Available Hardware */}
      <fieldset className="onboarding-fieldset">
        <legend>Available Hardware (select all that apply)</legend>
        {HARDWARE_OPTIONS.map(({ value, label }) => (
          <label key={value} className="onboarding-checkbox">
            <input
              type="checkbox"
              checked={availableHardware.includes(value)}
              onChange={() => toggleHardware(value)}
            />
            {label}
          </label>
        ))}
      </fieldset>

      {/* Learning Goal */}
      <label className="auth-form-label">
        Learning Goal
        <textarea
          value={learningGoal}
          onChange={(e) => setLearningGoal(e.target.value.slice(0, 500))}
          className="auth-form-input"
          placeholder="What do you hope to learn?"
          maxLength={500}
          rows={3}
        />
      </label>

      {/* Preferred Pace */}
      <fieldset className="onboarding-fieldset">
        <legend>Preferred Pace</legend>
        <label className="onboarding-radio">
          <input
            type="radio"
            name="preferred_pace"
            value="self_paced"
            checked={preferredPace === "self_paced"}
            onChange={() => setPreferredPace("self_paced")}
          />
          Self-paced
        </label>
        <label className="onboarding-radio">
          <input
            type="radio"
            name="preferred_pace"
            value="structured_weekly"
            checked={preferredPace === "structured_weekly"}
            onChange={() => setPreferredPace("structured_weekly")}
          />
          Structured Weekly
        </label>
      </fieldset>

      <div className="onboarding-actions">
        <button type="submit" disabled={isSubmitting} className="auth-form-button">
          {isSubmitting ? "Saving..." : isEdit ? "Update Profile" : "Complete Setup"}
        </button>
        {!isEdit && (
          <button
            type="button"
            onClick={handleSkip}
            disabled={isSubmitting}
            className="auth-form-link"
          >
            Skip for now
          </button>
        )}
      </div>
    </form>
  );
}
