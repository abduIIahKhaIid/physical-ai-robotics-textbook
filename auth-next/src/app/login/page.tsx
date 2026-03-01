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
    <main className="min-h-screen flex relative overflow-hidden"
      style={{ background: "#f8fafc" }}>

      {/* ── Animations ── */}
      <style>{`
        @keyframes float1 {
          0%, 100% { transform: translate(0, 0) rotate(0deg); }
          50% { transform: translate(15px, -20px) rotate(3deg); }
        }
        @keyframes float2 {
          0%, 100% { transform: translate(0, 0) rotate(0deg); }
          50% { transform: translate(-20px, 15px) rotate(-2deg); }
        }
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(24px); }
          to   { opacity: 1; transform: translateY(0); }
        }
        .anim-1 { opacity: 0; animation: slideUp 0.6s ease-out 0.1s forwards; }
        .anim-2 { opacity: 0; animation: slideUp 0.6s ease-out 0.2s forwards; }
        .anim-3 { opacity: 0; animation: slideUp 0.6s ease-out 0.3s forwards; }
      `}</style>

      {/* ── Left panel — brand / illustration ── */}
      <div className="hidden lg:flex lg:w-[45%] relative flex-col justify-between p-10 overflow-hidden"
        style={{ background: "linear-gradient(135deg, #1e3a8a 0%, #3b82f6 50%, #6366f1 100%)" }}>

        {/* Decorative shapes */}
        <div className="absolute -top-20 -right-20 w-72 h-72 rounded-full pointer-events-none"
          style={{ background: "rgba(255,255,255,0.08)", animation: "float1 20s ease-in-out infinite" }} />
        <div className="absolute -bottom-16 -left-16 w-56 h-56 rounded-full pointer-events-none"
          style={{ background: "rgba(255,255,255,0.06)", animation: "float2 24s ease-in-out infinite" }} />
        <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-96 h-96 rounded-full pointer-events-none"
          style={{ background: "rgba(255,255,255,0.04)" }} />

        {/* Brand */}
        <a href={homeUrl} className="relative z-10 flex items-center gap-3 no-underline">
          <div className="w-10 h-10 rounded-xl flex items-center justify-center text-lg font-bold"
            style={{ background: "rgba(255,255,255,0.2)", color: "white", backdropFilter: "blur(8px)" }}>
            R
          </div>
          <span className="text-white font-semibold text-lg tracking-tight">Physical AI Robotics</span>
        </a>

        {/* Center content */}
        <div className="relative z-10 flex-1 flex flex-col justify-center">
          <h2 className="text-white text-3xl font-bold leading-tight mb-4">
            Your gateway to<br />Physical AI mastery
          </h2>
          <p style={{ color: "rgba(255,255,255,0.8)" }} className="text-base leading-relaxed max-w-sm">
            Access comprehensive robotics courses, hands-on simulations, and an AI-powered learning assistant.
          </p>

          {/* Stats */}
          <div className="flex gap-8 mt-10">
            {[
              { value: "4", label: "Modules" },
              { value: "50+", label: "Chapters" },
              { value: "Free", label: "Open Source" },
            ].map((s) => (
              <div key={s.label}>
                <div className="text-white text-2xl font-bold">{s.value}</div>
                <div style={{ color: "rgba(255,255,255,0.7)" }} className="text-sm">{s.label}</div>
              </div>
            ))}
          </div>
        </div>

        {/* Bottom quote */}
        <div className="relative z-10">
          <p style={{ color: "rgba(255,255,255,0.6)" }} className="text-sm italic">
            &ldquo;The best way to predict the future is to build it.&rdquo;
          </p>
        </div>
      </div>

      {/* ── Right panel — form ── */}
      <div className="flex-1 flex flex-col items-center justify-center px-6 py-12 relative">

        {/* Back link */}
        <a href={homeUrl}
          className="anim-1 absolute top-6 left-6 lg:top-8 lg:left-8 flex items-center gap-1.5 text-sm font-medium no-underline"
          style={{ color: "#64748b" }}
          onMouseEnter={e => (e.currentTarget.style.color = "#2563eb")}
          onMouseLeave={e => (e.currentTarget.style.color = "#64748b")}
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24"
            fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M19 12H5M12 19l-7-7 7-7" />
          </svg>
          Back to Home
        </a>

        <div className="w-full max-w-[420px]">

          {/* Mobile logo */}
          <a href={homeUrl} className="anim-1 lg:hidden flex items-center justify-center gap-2.5 mb-8 no-underline">
            <div className="w-9 h-9 rounded-xl flex items-center justify-center text-base font-bold text-white"
              style={{ background: "linear-gradient(135deg, #3b82f6, #6366f1)" }}>
              R
            </div>
            <span className="font-semibold tracking-tight" style={{ color: "#0f172a" }}>
              Physical AI Robotics
            </span>
          </a>

          {/* Card */}
          <Card className="anim-2 rounded-2xl overflow-hidden"
            style={{
              background: "white",
              border: "1px solid #e2e8f0",
              boxShadow: "0 1px 3px rgba(0,0,0,0.04), 0 8px 32px rgba(0,0,0,0.06)",
            }}
          >
            <div style={{ height: 3, background: "linear-gradient(90deg, #3b82f6, #6366f1, #8b5cf6)" }} />

            <CardHeader className="pb-2 pt-8 px-8">
              <CardTitle className="text-2xl font-bold tracking-tight" style={{ color: "#0f172a" }}>
                Welcome back
              </CardTitle>
              <CardDescription className="text-sm" style={{ color: "#64748b" }}>
                Sign in to continue your learning journey
              </CardDescription>
            </CardHeader>

            <CardContent className="px-8 pt-4">
              <form onSubmit={handleSubmit} className="flex flex-col gap-5">
                <div className="flex flex-col gap-2">
                  <Label htmlFor="email" className="text-sm font-medium" style={{ color: "#374151" }}>
                    Email
                  </Label>
                  <Input
                    id="email"
                    type="email"
                    placeholder="you@example.com"
                    value={email}
                    onChange={e => setEmail(e.target.value)}
                    required
                    className="rounded-xl h-12 border transition-all duration-200 focus-visible:ring-2 focus-visible:ring-blue-500/30"
                    style={{
                      background: "#f8fafc",
                      borderColor: "#e2e8f0",
                      color: "#0f172a",
                    }}
                    onFocus={e => {
                      e.currentTarget.style.borderColor = "#3b82f6";
                      e.currentTarget.style.background = "#fff";
                    }}
                    onBlur={e => {
                      e.currentTarget.style.borderColor = "#e2e8f0";
                      e.currentTarget.style.background = "#f8fafc";
                    }}
                  />
                </div>

                <div className="flex flex-col gap-2">
                  <Label htmlFor="password" className="text-sm font-medium" style={{ color: "#374151" }}>
                    Password
                  </Label>
                  <Input
                    id="password"
                    type="password"
                    placeholder="••••••••"
                    value={password}
                    onChange={e => setPassword(e.target.value)}
                    required
                    className="rounded-xl h-12 border transition-all duration-200 focus-visible:ring-2 focus-visible:ring-blue-500/30"
                    style={{
                      background: "#f8fafc",
                      borderColor: "#e2e8f0",
                      color: "#0f172a",
                    }}
                    onFocus={e => {
                      e.currentTarget.style.borderColor = "#3b82f6";
                      e.currentTarget.style.background = "#fff";
                    }}
                    onBlur={e => {
                      e.currentTarget.style.borderColor = "#e2e8f0";
                      e.currentTarget.style.background = "#f8fafc";
                    }}
                  />
                </div>

                {error && (
                  <p className="text-sm rounded-xl px-4 py-2.5"
                    style={{ color: "#dc2626", background: "#fef2f2", border: "1px solid #fecaca" }}>
                    {error}
                  </p>
                )}

                <Button
                  type="submit"
                  disabled={loading}
                  className="h-12 rounded-xl font-semibold text-white border-0 mt-1 cursor-pointer disabled:opacity-50 transition-all duration-300 hover:shadow-lg hover:-translate-y-0.5"
                  style={{
                    background: "linear-gradient(135deg, #2563eb, #4f46e5)",
                    boxShadow: "0 2px 8px rgba(37,99,235,0.25)",
                  }}
                >
                  {loading ? (
                    <span className="flex items-center gap-2">
                      <svg className="animate-spin h-4 w-4" viewBox="0 0 24 24" fill="none">
                        <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                        <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4z" />
                      </svg>
                      Signing in...
                    </span>
                  ) : "Sign in"}
                </Button>
              </form>
            </CardContent>

            <CardFooter className="pt-4 pb-8 px-8 justify-center">
              <p className="text-sm" style={{ color: "#64748b" }}>
                No account?{" "}
                <Link href="/register" className="font-semibold no-underline"
                  style={{ color: "#2563eb" }}
                  onMouseEnter={e => (e.currentTarget.style.color = "#1d4ed8")}
                  onMouseLeave={e => (e.currentTarget.style.color = "#2563eb")}
                >
                  Create one free
                </Link>
              </p>
            </CardFooter>
          </Card>

          {/* Trust indicators */}
          <div className="anim-3 flex items-center justify-center gap-6 mt-6">
            {[
              { icon: "M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z", text: "Secure" },
              { icon: "M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5", text: "Free" },
            ].map((item) => (
              <div key={item.text} className="flex items-center gap-1.5">
                <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24"
                  fill="none" stroke="#94a3b8" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d={item.icon} />
                </svg>
                <span className="text-xs" style={{ color: "#94a3b8" }}>{item.text}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </main>
  );
}
