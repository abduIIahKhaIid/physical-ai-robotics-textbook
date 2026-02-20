import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import { env } from "./env.js";

const app = express();

// CORS — allow frontend origin
app.use(
  cors({
    origin: env.FRONTEND_URL,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    credentials: true,
  })
);

// Mount Better-Auth handler on all /api/auth/* routes
// IMPORTANT: Must be before express.json() to avoid body parsing conflicts
app.all("/api/auth/*", toNodeHandler(auth));

// Health check
app.get("/health", (_req, res) => {
  res.json({ status: "ok", service: "auth-service" });
});

app.listen(env.BETTER_AUTH_PORT, () => {
  console.log(`Auth service listening on port ${env.BETTER_AUTH_PORT}`);
});
