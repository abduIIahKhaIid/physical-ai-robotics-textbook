"""Pydantic models for user profile (onboarding questionnaire)."""

from __future__ import annotations

from datetime import datetime
from typing import Literal, Optional

from pydantic import BaseModel, Field

HardwareOption = Literal[
    "jetson_nano_orin",
    "raspberry_pi",
    "ros2_workstation",
    "gpu_workstation",
    "simulation_only",
]


class ProfileCreate(BaseModel):
    """Request body for creating/replacing a user profile."""

    software_level: Literal["beginner", "intermediate", "advanced"]
    programming_languages: str = Field(default="", max_length=200)
    hardware_level: Literal["none", "hobbyist", "academic", "professional"]
    available_hardware: list[HardwareOption] = Field(default_factory=list)
    learning_goal: str = Field(default="", max_length=500)
    preferred_pace: Literal["self_paced", "structured_weekly"]


class ProfileUpdate(BaseModel):
    """Request body for partially updating a user profile. All fields optional."""

    software_level: Optional[Literal["beginner", "intermediate", "advanced"]] = None
    programming_languages: Optional[str] = Field(default=None, max_length=200)
    hardware_level: Optional[Literal["none", "hobbyist", "academic", "professional"]] = None
    available_hardware: Optional[list[HardwareOption]] = None
    learning_goal: Optional[str] = Field(default=None, max_length=500)
    preferred_pace: Optional[Literal["self_paced", "structured_weekly"]] = None


class ProfileResponse(BaseModel):
    """Response body for profile endpoints."""

    user_id: str
    software_level: str
    programming_languages: str
    hardware_level: str
    available_hardware: list[str]
    learning_goal: str
    preferred_pace: str
    onboarding_completed: bool
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None
