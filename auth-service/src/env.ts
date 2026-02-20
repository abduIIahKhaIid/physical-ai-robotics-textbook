import dotenv from "dotenv";

dotenv.config();

function requireEnv(name: string): string {
  const value = process.env[name];
  if (!value) {
    throw new Error(`Missing required environment variable: ${name}`);
  }
  return value;
}

export const env = {
  DATABASE_URL: requireEnv("DATABASE_URL"),
  BETTER_AUTH_SECRET: requireEnv("BETTER_AUTH_SECRET"),
  BETTER_AUTH_PORT: parseInt(process.env.BETTER_AUTH_PORT || "4000", 10),
  FRONTEND_URL: process.env.FRONTEND_URL || "http://localhost:3000",
} as const;
