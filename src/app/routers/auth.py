# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Authentication API endpoints.

Provides login, token refresh, and user info endpoints.
Only active when auth_enabled=True in config.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel

from app.auth import (
    authenticate_user,
    create_access_token,
    create_refresh_token,
    decode_token,
    require_auth,
)
from app.config import settings

router = APIRouter(prefix="/api/auth", tags=["auth"])


class LoginRequest(BaseModel):
    username: str
    password: str


class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int


class RefreshRequest(BaseModel):
    refresh_token: str


@router.post("/login", response_model=TokenResponse)
async def login(request: LoginRequest):
    """Authenticate and receive JWT tokens."""
    if not settings.auth_enabled:
        # Auth disabled — return a dummy token
        return TokenResponse(
            access_token=create_access_token("admin", "admin"),
            refresh_token=create_refresh_token("admin"),
            expires_in=settings.auth_access_token_expire_minutes * 60,
        )

    user = authenticate_user(request.username, request.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid username or password",
        )

    return TokenResponse(
        access_token=create_access_token(user["sub"], user["role"]),
        refresh_token=create_refresh_token(user["sub"]),
        expires_in=settings.auth_access_token_expire_minutes * 60,
    )


@router.post("/refresh", response_model=TokenResponse)
async def refresh(request: RefreshRequest):
    """Refresh an access token using a refresh token."""
    payload = decode_token(request.refresh_token)
    if payload.get("type") != "refresh":
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid refresh token",
        )

    return TokenResponse(
        access_token=create_access_token(payload["sub"], payload.get("role", "user")),
        refresh_token=create_refresh_token(payload["sub"]),
        expires_in=settings.auth_access_token_expire_minutes * 60,
    )


@router.get("/me")
async def get_current_user(user: dict = Depends(require_auth)):
    """Get current user information."""
    return {
        "username": user["sub"],
        "role": user.get("role", "user"),
    }


@router.get("/status")
async def auth_status():
    """Check if authentication is enabled."""
    return {
        "auth_enabled": settings.auth_enabled,
        "tls_enabled": settings.tls_enabled,
    }
