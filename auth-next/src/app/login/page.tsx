"use client";

import { useState } from "react";
import Link from "next/link";
import { useRouter } from "next/navigation";
import { signIn } from "@/lib/auth-client";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";

export default function LoginPage() {
  const router = useRouter();
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const redirectUrl =
    process.env.NEXT_PUBLIC_REDIRECT_URL ||
    "https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/";

  const homeUrl =
    process.env.NEXT_PUBLIC_HOME_URL ||
    "https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/";

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    setError("");
    setLoading(true);
    try {
      const result = await signIn.email({ email, password });
      if (result.error) {
        setError(result.error.message || "Invalid email or password.");
      } else {
        router.push(redirectUrl);
      }
    } catch {
      setError("Something went wrong. Please try again.");
    } finally {
      setLoading(false);
    }
  }

  return (
    <main className="min-h-screen bg-[#0f172a] flex flex-col items-center justify-center px-4 relative overflow-hidden">
      {/* Background glows matching website blue→purple theme */}
      <div className="absolute top-0 left-1/4 w-[500px] h-[500px] bg-blue-600/10 rounded-full blur-[120px] pointer-events-none" />
      <div className="absolute bottom-0 right-1/4 w-[400px] h-[400px] bg-purple-700/10 rounded-full blur-[120px] pointer-events-none" />

      {/* Back to Home */}
      <a
        href={homeUrl}
        className="absolute top-6 left-6 flex items-center gap-1.5 text-white/40 hover:text-white/80 text-sm transition-colors z-10"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M19 12H5M12 19l-7-7 7-7" />
        </svg>
        Back to Home
      </a>

      <div className="relative z-10 w-full max-w-md">
        {/* Logo */}
        <a
          href={homeUrl}
          className="flex items-center justify-center gap-2.5 mb-8"
        >
          <div className="w-9 h-9 rounded-xl bg-gradient-to-br from-blue-500 to-purple-600 flex items-center justify-center text-base font-bold text-white shadow-lg shadow-blue-900/30">
            R
          </div>
          <span className="font-semibold text-white tracking-tight">
            Physical AI Robotics
          </span>
        </a>

        <Card className="bg-white/5 border border-white/10 text-white rounded-2xl shadow-2xl shadow-black/50">
          <CardHeader className="pb-4">
            <CardTitle className="text-2xl font-bold text-white">
              Welcome back
            </CardTitle>
            <CardDescription className="text-white/50">
              Sign in to continue to the textbook
            </CardDescription>
          </CardHeader>

          <CardContent>
            <form onSubmit={handleSubmit} className="flex flex-col gap-4">
              <div className="flex flex-col gap-1.5">
                <Label htmlFor="email" className="text-white/70 text-sm">
                  Email
                </Label>
                <Input
                  id="email"
                  type="email"
                  placeholder="you@example.com"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  className="bg-white/8 border-white/15 text-white placeholder:text-white/30 focus-visible:ring-blue-500 focus-visible:border-blue-500/50 rounded-xl h-11"
                />
              </div>

              <div className="flex flex-col gap-1.5">
                <Label htmlFor="password" className="text-white/70 text-sm">
                  Password
                </Label>
                <Input
                  id="password"
                  type="password"
                  placeholder="••••••••"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  className="bg-white/8 border-white/15 text-white placeholder:text-white/30 focus-visible:ring-blue-500 focus-visible:border-blue-500/50 rounded-xl h-11"
                />
              </div>

              {error && (
                <p className="text-sm text-red-400 bg-red-500/10 border border-red-500/20 rounded-lg px-3 py-2">
                  {error}
                </p>
              )}

              <Button
                type="submit"
                disabled={loading}
                className="bg-gradient-to-r from-blue-600 to-purple-600 hover:from-blue-500 hover:to-purple-500 text-white rounded-xl h-11 font-semibold mt-1 cursor-pointer disabled:opacity-50 shadow-lg shadow-blue-900/30 border-0"
              >
                {loading ? "Signing in…" : "Sign in"}
              </Button>
            </form>
          </CardContent>

          <CardFooter className="pt-0 justify-center">
            <p className="text-sm text-white/40">
              No account?{" "}
              <Link
                href="/register"
                className="text-blue-400 hover:text-blue-300 font-medium"
              >
                Create one free
              </Link>
            </p>
          </CardFooter>
        </Card>
      </div>
    </main>
  );
}
