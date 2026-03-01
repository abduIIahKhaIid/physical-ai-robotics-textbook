import { betterAuth } from "better-auth";
import { bearer } from "better-auth/plugins";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL!,
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  session: {
    expiresIn: 86400,
  },
  secret: process.env.BETTER_AUTH_SECRET!,
  trustedOrigins: [
    process.env.NEXT_PUBLIC_APP_URL || "http://localhost:3000",
    process.env.NEXT_PUBLIC_REDIRECT_URL || "http://localhost:3000",
  ],
  plugins: [bearer()],
});
