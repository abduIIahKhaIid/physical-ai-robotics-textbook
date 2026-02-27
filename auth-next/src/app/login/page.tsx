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
    <main
      style={{ background: "linear-gradient(180deg, #000000 0%, #0f172a 100%)" }}
      className="min-h-screen flex flex-col items-center justify-center px-4 relative overflow-hidden"
    >
      {/* ── Blobs matching homepage ── */}
      <div
        className="absolute rounded-full filter blur-[100px] pointer-events-none"
        style={{
          width: 500, height: 500,
          top: "10%", left: "15%",
          background: "rgba(139, 92, 246, 0.12)",
          animation: "blob1 18s ease-in-out infinite",
        }}
      />
      <div
        className="absolute rounded-full filter blur-[120px] pointer-events-none"
        style={{
          width: 600, height: 600,
          bottom: "10%", right: "10%",
          background: "rgba(59, 130, 246, 0.10)",
          animation: "blob2 22s ease-in-out infinite",
        }}
      />
      <div
        className="absolute rounded-full filter blur-[80px] pointer-events-none"
        style={{
          width: 400, height: 400,
          top: "55%", left: "5%",
          background: "rgba(6, 182, 212, 0.08)",
          animation: "blob3 26s ease-in-out infinite",
        }}
      />
      <div
        className="absolute rounded-full filter blur-[90px] pointer-events-none"
        style={{
          width: 350, height: 350,
          top: "5%", right: "20%",
          background: "rgba(99, 102, 241, 0.06)",
          animation: "blob4 20s ease-in-out infinite",
        }}
      />

      {/* ── Blob + entrance animations ── */}
      <style>{`
        @keyframes blob1 {
          0%, 100% { transform: translate(0, 0) scale(1); }
          33% { transform: translate(30px, -20px) scale(1.05); }
          66% { transform: translate(-20px, 10px) scale(0.97); }
        }
        @keyframes blob2 {
          0%, 100% { transform: translate(0, 0) scale(1); }
          40% { transform: translate(-25px, 15px) scale(1.04); }
          70% { transform: translate(15px, -25px) scale(0.98); }
        }
        @keyframes blob3 {
          0%, 100% { transform: translate(0, 0) scale(1); }
          50% { transform: translate(20px, 20px) scale(1.06); }
        }
        @keyframes blob4 {
          0%, 100% { transform: translate(0, 0) scale(1); }
          45% { transform: translate(-15px, -15px) scale(1.03); }
        }
        @keyframes heroEnter {
          from { opacity: 0; transform: translateY(20px) scale(0.98); }
          to   { opacity: 1; transform: translateY(0)    scale(1);    }
        }
        .auth-card { opacity: 0; animation: heroEnter 0.8s cubic-bezier(0.16,1,0.3,1) 0.35s forwards; }
        .auth-logo { opacity: 0; animation: heroEnter 0.8s cubic-bezier(0.16,1,0.3,1) 0.15s forwards; }
        .auth-back { opacity: 0; animation: heroEnter 0.8s cubic-bezier(0.16,1,0.3,1) 0.05s forwards; }
      `}</style>

      {/* ── Back to Home ── */}
      <a
        href={homeUrl}
        className="auth-back absolute top-6 left-6 flex items-center gap-1.5 text-sm transition-colors z-10"
        style={{ color: "rgba(148,163,184,0.7)" }}
        onMouseEnter={e => (e.currentTarget.style.color = "#93c5fd")}
        onMouseLeave={e => (e.currentTarget.style.color = "rgba(148,163,184,0.7)")}
      >
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24"
          fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M19 12H5M12 19l-7-7 7-7" />
        </svg>
        Back to Home
      </a>

      <div className="relative z-10 w-full max-w-md">

        {/* ── Logo / Brand ── */}
        <a href={homeUrl} className="auth-logo flex items-center justify-center gap-2.5 mb-8 no-underline">
          <div
            className="w-9 h-9 rounded-xl flex items-center justify-center text-base font-bold text-white"
            style={{
              background: "linear-gradient(135deg, #3b82f6, #6366f1)",
              boxShadow: "0 8px 24px rgba(59,130,246,0.3)",
            }}
          >
            R
          </div>
          <span className="font-semibold tracking-tight" style={{ color: "#f0f9ff" }}>
            Physical AI Robotics
          </span>
        </a>

        {/* ── Badge ── */}
        <div className="auth-logo flex justify-center mb-6">
          <span
            className="inline-flex items-center gap-2 px-5 py-2 rounded-full text-sm font-medium backdrop-blur-md"
            style={{
              background: "rgba(30, 58, 138, 0.35)",
              color: "#93c5fd",
              border: "1px solid rgba(59, 130, 246, 0.3)",
            }}
          >
            <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24"
              fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2" />
            </svg>
            Sign in to access the textbook
          </span>
        </div>

        {/* ── Card ── */}
        <Card
          className="auth-card rounded-2xl shadow-2xl"
          style={{
            background: "rgba(15, 23, 42, 0.6)",
            border: "1px solid rgba(148, 163, 184, 0.15)",
            backdropFilter: "blur(12px)",
            boxShadow: "0 1px 3px rgba(0,0,0,0.3), 0 8px 24px rgba(59,130,246,0.08)",
          }}
        >
          <CardHeader className="pb-4">
            <CardTitle className="text-2xl font-extrabold tracking-tight" style={{ color: "#f0f9ff" }}>
              Welcome back
            </CardTitle>
            <CardDescription style={{ color: "#cbd5e1" }}>
              Sign in to continue your learning journey
            </CardDescription>
          </CardHeader>

          <CardContent>
            <form onSubmit={handleSubmit} className="flex flex-col gap-4">
              <div className="flex flex-col gap-1.5">
                <Label htmlFor="email" className="text-sm font-medium" style={{ color: "#94a3b8" }}>
                  Email
                </Label>
                <Input
                  id="email"
                  type="email"
                  placeholder="you@example.com"
                  value={email}
                  onChange={e => setEmail(e.target.value)}
                  required
                  className="rounded-xl h-11 text-white placeholder:text-slate-500 border-0 focus-visible:ring-2 focus-visible:ring-blue-500/50"
                  style={{ background: "rgba(30, 41, 59, 0.8)" }}
                />
              </div>

              <div className="flex flex-col gap-1.5">
                <Label htmlFor="password" className="text-sm font-medium" style={{ color: "#94a3b8" }}>
                  Password
                </Label>
                <Input
                  id="password"
                  type="password"
                  placeholder="••••••••"
                  value={password}
                  onChange={e => setPassword(e.target.value)}
                  required
                  className="rounded-xl h-11 text-white placeholder:text-slate-500 border-0 focus-visible:ring-2 focus-visible:ring-blue-500/50"
                  style={{ background: "rgba(30, 41, 59, 0.8)" }}
                />
              </div>

              {error && (
                <p className="text-sm text-red-400 rounded-lg px-3 py-2"
                  style={{ background: "rgba(239,68,68,0.1)", border: "1px solid rgba(239,68,68,0.2)" }}>
                  {error}
                </p>
              )}

              <Button
                type="submit"
                disabled={loading}
                className="h-11 rounded-2xl font-semibold text-white border-0 mt-1 cursor-pointer disabled:opacity-50 transition-all duration-300 hover:scale-[1.02] hover:-translate-y-0.5 focus:ring-4 focus:ring-blue-400/30"
                style={{
                  background: "linear-gradient(135deg, #2563eb, #4f46e5)",
                  boxShadow: "0 1px 2px rgba(0,0,0,0.3), 0 8px 24px rgba(37,99,235,0.3)",
                }}
              >
                {loading ? "Signing in…" : "Sign in →"}
              </Button>
            </form>
          </CardContent>

          <CardFooter className="pt-0 justify-center">
            <p className="text-sm" style={{ color: "rgba(148,163,184,0.7)" }}>
              No account?{" "}
              <Link href="/register" className="font-medium transition-colors" style={{ color: "#93c5fd" }}>
                Create one free
              </Link>
            </p>
          </CardFooter>
        </Card>
      </div>
    </main>
  );
}
