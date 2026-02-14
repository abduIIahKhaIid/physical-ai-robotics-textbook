import React, { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {
  Sparkles, Cpu, Bot, Cog, BrainCircuit, Atom,
  ArrowRight, BookOpen, ChevronDown,
} from 'lucide-react';
import { HERO_COUNTERS, COURSE_OVERVIEW } from '../../data/courseData';
import styles from './HeroSection.module.css';

const counterColors = [
  { text: 'text-blue-600 dark:text-blue-400', glow: 'from-blue-400/20 to-cyan-400/20', border: 'hover:border-blue-400 dark:hover:border-blue-500' },
  { text: 'text-purple-600 dark:text-purple-400', glow: 'from-purple-400/20 to-pink-400/20', border: 'hover:border-purple-400 dark:hover:border-purple-500' },
  { text: 'text-cyan-600 dark:text-cyan-400', glow: 'from-cyan-400/20 to-teal-400/20', border: 'hover:border-cyan-400 dark:hover:border-cyan-500' },
  { text: 'text-green-600 dark:text-green-400', glow: 'from-green-400/20 to-emerald-400/20', border: 'hover:border-green-400 dark:hover:border-green-500' },
];

export default function HeroSection({ isDarkTheme }) {
  const canvasRef = useRef(null);
  const heroContentRef = useRef(null);
  const [activeCard, setActiveCard] = useState(null);
  const [counters, setCounters] = useState(
    Object.fromEntries(HERO_COUNTERS.map(c => [c.key, 0]))
  );
  const [isVisible, setIsVisible] = useState(false);

  // Particle network animation
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animationFrameId;
    let particles;
    const mouse = { x: null, y: null, radius: 120 };

    const resizeCanvas = () => {
      canvas.width = window.innerWidth;
      canvas.height = canvas.parentElement?.offsetHeight || window.innerHeight;
    };

    class Particle {
      constructor(x, y) {
        this.x = x;
        this.y = y;
        this.size = Math.random() * 1.5 + 0.5;
        this.baseX = x;
        this.baseY = y;
        this.density = (Math.random() * 30) + 1;
        this.vx = (Math.random() - 0.5) * 0.3;
        this.vy = (Math.random() - 0.5) * 0.3;
      }

      draw() {
        ctx.fillStyle = isDarkTheme
          ? `rgba(147, 197, 253, ${0.3 + this.size * 0.2})`
          : `rgba(59, 130, 246, ${0.25 + this.size * 0.15})`;
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
        ctx.closePath();
        ctx.fill();
      }

      update() {
        this.baseX += this.vx;
        this.baseY += this.vy;
        if (this.baseX < 0 || this.baseX > canvas.width) this.vx *= -1;
        if (this.baseY < 0 || this.baseY > canvas.height) this.vy *= -1;

        if (mouse.x !== null && mouse.y !== null) {
          const dx = mouse.x - this.x;
          const dy = mouse.y - this.y;
          const distance = Math.sqrt(dx * dx + dy * dy);
          if (distance < mouse.radius) {
            const force = (mouse.radius - distance) / mouse.radius;
            this.x -= (dx / distance) * force * this.density * 0.6;
            this.y -= (dy / distance) * force * this.density * 0.6;
          } else {
            this.x += (this.baseX - this.x) * 0.05;
            this.y += (this.baseY - this.y) * 0.05;
          }
        } else {
          this.x += (this.baseX - this.x) * 0.05;
          this.y += (this.baseY - this.y) * 0.05;
        }
      }
    }

    const init = () => {
      particles = [];
      const count = Math.min((canvas.width * canvas.height) / 8000, 200);
      for (let i = 0; i < count; i++) {
        particles.push(new Particle(Math.random() * canvas.width, Math.random() * canvas.height));
      }
    };

    const connect = () => {
      for (let a = 0; a < particles.length; a++) {
        for (let b = a + 1; b < particles.length; b++) {
          const dx = particles[a].x - particles[b].x;
          const dy = particles[a].y - particles[b].y;
          const distance = Math.sqrt(dx * dx + dy * dy);
          if (distance < 130) {
            const opacity = (1 - distance / 130) * 0.35;
            ctx.strokeStyle = isDarkTheme
              ? `rgba(147, 197, 253, ${opacity})`
              : `rgba(59, 130, 246, ${opacity * 0.7})`;
            ctx.lineWidth = 0.6;
            ctx.beginPath();
            ctx.moveTo(particles[a].x, particles[a].y);
            ctx.lineTo(particles[b].x, particles[b].y);
            ctx.stroke();
          }
        }
      }
    };

    const animate = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      particles.forEach(p => { p.update(); p.draw(); });
      connect();
      animationFrameId = requestAnimationFrame(animate);
    };

    const onMouseMove = (e) => {
      const rect = canvas.getBoundingClientRect();
      mouse.x = e.clientX - rect.left;
      mouse.y = e.clientY - rect.top;
    };
    const onMouseLeave = () => { mouse.x = null; mouse.y = null; };
    const onResize = () => { resizeCanvas(); init(); };

    resizeCanvas(); init(); animate();
    window.addEventListener('resize', onResize);
    canvas.addEventListener('mousemove', onMouseMove);
    canvas.addEventListener('mouseleave', onMouseLeave);

    return () => {
      window.removeEventListener('resize', onResize);
      canvas.removeEventListener('mousemove', onMouseMove);
      canvas.removeEventListener('mouseleave', onMouseLeave);
      cancelAnimationFrame(animationFrameId);
    };
  }, [isDarkTheme]);

  // 3D tilt on content
  useEffect(() => {
    const el = heroContentRef.current;
    if (!el) return;

    const onMove = (e) => {
      const rect = el.getBoundingClientRect();
      const x = ((e.clientX - rect.left) / rect.width - 0.5) * 2;
      const y = ((e.clientY - rect.top) / rect.height - 0.5) * 2;
      el.style.transform = `perspective(1000px) rotateY(${x * 2}deg) rotateX(${-y * 2}deg)`;
    };
    const onLeave = () => {
      el.style.transform = 'perspective(1000px) rotateY(0deg) rotateX(0deg)';
    };

    el.addEventListener('mousemove', onMove);
    el.addEventListener('mouseleave', onLeave);
    return () => {
      el.removeEventListener('mousemove', onMove);
      el.removeEventListener('mouseleave', onLeave);
    };
  }, []);

  // Animated counter effect - uses HERO_COUNTERS from courseData
  useEffect(() => {
    if (!isVisible) return;

    const duration = 2000;
    const startTime = Date.now();
    const targets = Object.fromEntries(HERO_COUNTERS.map(c => [c.key, c.target]));

    const interval = setInterval(() => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min(elapsed / duration, 1);
      const easeProgress = 1 - Math.pow(1 - progress, 3);

      const newCounters = {};
      for (const [key, target] of Object.entries(targets)) {
        newCounters[key] = Math.floor(target * easeProgress);
      }
      setCounters(newCounters);

      if (progress === 1) clearInterval(interval);
    }, 16);

    return () => clearInterval(interval);
  }, [isVisible]);

  // Intersection observer
  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.unobserve(entry.target);
        }
      },
      { threshold: 0.3 }
    );

    if (heroContentRef.current) {
      observer.observe(heroContentRef.current);
    }

    return () => observer.disconnect();
  }, []);

  // Keyboard navigation
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'ArrowDown') {
        e.preventDefault();
        scrollToContent();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  const scrollToContent = () => {
    const nextSection = document.querySelector('#curriculum');
    nextSection?.scrollIntoView({ behavior: 'smooth' });
  };

  return (
    <section className={clsx("relative min-h-screen flex items-center overflow-hidden", styles.heroBg)}>

      {/* Gradient mesh background */}
      <div className="absolute inset-0 z-0 overflow-hidden" aria-hidden="true">
        <div className={clsx("absolute w-[500px] h-[500px] rounded-full filter blur-[100px] top-[10%] left-[15%]", styles.animateBlob1, styles.heroBlob1)} />
        <div className={clsx("absolute w-[600px] h-[600px] rounded-full filter blur-[120px] bottom-[10%] right-[10%]", styles.animateBlob2, styles.heroBlob2)} />
        <div className={clsx("absolute w-[400px] h-[400px] rounded-full filter blur-[80px] top-[55%] left-[5%]", styles.animateBlob3, styles.heroBlob3)} />
        <div className={clsx("absolute w-[350px] h-[350px] rounded-full filter blur-[90px] top-[5%] right-[20%]", styles.animateBlob4, styles.heroBlob4)} />
        <div className="absolute inset-x-0 top-0 h-32 bg-gradient-to-b from-white dark:from-black to-transparent" />
        <div className="absolute inset-x-0 bottom-0 h-32 bg-gradient-to-t from-gray-50 dark:from-gray-950 to-transparent" />
      </div>

      {/* Floating decorative icons */}
      <div className="absolute inset-0 z-[5] overflow-hidden pointer-events-none hidden lg:block" aria-hidden="true">
        <Cpu className={clsx("absolute top-[14%] left-[7%] w-9 h-9", styles.heroFloatIcon, "animate-float")} style={{ animationDelay: '0s' }} />
        <Bot className={clsx("absolute top-[20%] right-[9%] w-12 h-12", styles.heroFloatIcon, "animate-float")} style={{ animationDelay: '1.2s' }} />
        <Cog className={clsx("absolute bottom-[28%] left-[10%] w-11 h-11", styles.heroFloatIcon, "animate-float")} style={{ animationDelay: '2.4s' }} />
        <BrainCircuit className={clsx("absolute bottom-[20%] right-[8%] w-9 h-9", styles.heroFloatIcon, "animate-float")} style={{ animationDelay: '3.6s' }} />
        <Atom className={clsx("absolute top-[52%] left-[4%] w-8 h-8", styles.heroFloatIcon, "animate-float")} style={{ animationDelay: '4.8s' }} />
      </div>

      {/* Particle canvas */}
      <canvas ref={canvasRef} className={clsx("absolute inset-0 z-10 w-full h-full", styles.parcelCanvas)} />

      {/* Content */}
      <div
        ref={heroContentRef}
        style={{ transition: 'transform 0.15s ease-out' }}
        className={clsx("relative z-20 container mx-auto px-6 lg:px-8 py-20 md:py-28 flex flex-col items-center justify-center text-center", styles.contentContainer)}
      >
        {/* Badge */}
        <div className={clsx("mb-6 inline-flex items-center gap-2 px-5 py-2.5 rounded-full text-sm font-medium backdrop-blur-md", styles.heroBadge, styles.heroStagger, styles.stagger1)}>
          <Sparkles className="w-4 h-4" />
          <span>Open Source & Community Driven</span>
        </div>

        {/* Main heading */}
        <h1 className={clsx("max-w-5xl text-[2.25rem] sm:text-4xl md:text-5xl lg:text-6xl xl:text-7xl font-extrabold tracking-tight leading-[1.1] mb-6", styles.heroHeading, styles.heroStagger, styles.stagger2)}>
          Master {COURSE_OVERVIEW.title}
        </h1>

        {/* Subtitle */}
        <p className={clsx("max-w-2xl mx-auto text-base sm:text-lg md:text-xl leading-relaxed mb-10", styles.heroSubtitle, styles.heroStagger, styles.stagger3)}>
          {COURSE_OVERVIEW.description}
        </p>

        {/* Interactive stats - 4 columns */}
        <div className={clsx("max-w-4xl mx-auto mb-12 grid grid-cols-2 md:grid-cols-4 gap-4 md:gap-6", styles.heroStagger, styles.stagger3)}>
          {HERO_COUNTERS.map((counter, index) => {
            const color = counterColors[index];
            return (
              <div key={counter.key} className={clsx("group relative", styles.interactiveStatCard)}>
                <div className={`absolute inset-0 bg-gradient-to-br ${color.glow} rounded-xl blur-lg group-hover:blur-xl transition-all duration-300 opacity-0 group-hover:opacity-100`} />
                <div className={`relative bg-white/50 dark:bg-gray-900/50 backdrop-blur-sm border border-gray-200 dark:border-gray-800 rounded-xl p-4 ${color.border} transition-all duration-300`}>
                  <div className={`text-3xl md:text-4xl font-bold ${color.text} mb-1`}>
                    {counters[counter.key]}{counter.suffix}
                  </div>
                  <div className="text-xs md:text-sm text-gray-600 dark:text-gray-400 font-medium">{counter.label}</div>
                </div>
              </div>
            );
          })}
        </div>

        {/* Key features */}
        <div className={clsx("max-w-2xl mx-auto mb-12 flex flex-wrap justify-center gap-4 text-sm text-gray-600 dark:text-gray-300", styles.heroStagger, styles.stagger3)}>
          <span
            className="group inline-flex items-center gap-1.5 px-3 py-1.5 bg-blue-100/80 dark:bg-blue-900/30 rounded-full hover:bg-blue-200 dark:hover:bg-blue-900/50 transition-all duration-300 cursor-pointer hover:scale-105 hover:shadow-lg"
            onMouseEnter={() => setActiveCard(0)}
            onMouseLeave={() => setActiveCard(null)}
          >
            <BrainCircuit className="w-4 h-4 group-hover:animate-spin" />
            Embodied Intelligence
          </span>
          <span
            className="group inline-flex items-center gap-1.5 px-3 py-1.5 bg-purple-100/80 dark:bg-purple-900/30 rounded-full hover:bg-purple-200 dark:hover:bg-purple-900/50 transition-all duration-300 cursor-pointer hover:scale-105 hover:shadow-lg"
            onMouseEnter={() => setActiveCard(1)}
            onMouseLeave={() => setActiveCard(null)}
          >
            <Cpu className="w-4 h-4 group-hover:animate-pulse" />
            Sim-to-Real Transfer
          </span>
          <span
            className="group inline-flex items-center gap-1.5 px-3 py-1.5 bg-cyan-100/80 dark:bg-cyan-900/30 rounded-full hover:bg-cyan-200 dark:hover:bg-cyan-900/50 transition-all duration-300 cursor-pointer hover:scale-105 hover:shadow-lg"
            onMouseEnter={() => setActiveCard(2)}
            onMouseLeave={() => setActiveCard(null)}
          >
            <Bot className="w-4 h-4 group-hover:animate-bounce" />
            Hands-on Labs
          </span>
        </div>

        {/* CTA Buttons */}
        <div className={clsx("flex flex-col sm:flex-row items-center justify-center gap-4", styles.heroStagger, styles.stagger4)}>
          <Link
            to="/docs/module-1/"
            className={clsx(
              'group w-full sm:w-auto inline-flex items-center justify-center gap-2.5 px-9 py-4 text-base font-semibold text-white rounded-2xl no-underline',
              styles.heroBtnPrimary,
              styles.interactiveBtn,
              'transition-all duration-300',
              'hover:scale-[1.03] hover:-translate-y-0.5 hover:text-white hover:no-underline',
              'focus:outline-none focus:ring-4 focus:ring-blue-400/30',
              'relative overflow-hidden'
            )}
          >
            <span className="relative z-10 flex items-center gap-2.5">
              Start Learning
              <ArrowRight className="w-5 h-5 transition-transform duration-300 group-hover:translate-x-1" />
            </span>
          </Link>
          <Link
            to="/docs/syllabus"
            className={clsx(
              'group w-full sm:w-auto inline-flex items-center justify-center gap-2.5 px-9 py-4 text-base font-semibold rounded-2xl no-underline',
              styles.heroBtnSecondary,
              styles.interactiveBtn,
              'transition-all duration-300',
              'hover:scale-[1.03] hover:-translate-y-0.5 hover:no-underline',
              'focus:outline-none focus:ring-4 focus:ring-gray-400/20',
              'relative overflow-hidden'
            )}
          >
            <span className="relative z-10 flex items-center gap-2.5">
              <BookOpen className="w-5 h-5" />
              View Syllabus
            </span>
          </Link>
        </div>
      </div>

      {/* Scroll indicator */}
      <div className={clsx("absolute bottom-8 left-1/2 -translate-x-1/2 z-20", styles.heroStagger, styles.stagger5)}>
        <button
          onClick={scrollToContent}
          className={clsx("flex flex-col items-center gap-1.5", styles.heroScrollBtn, "transition-colors duration-300 cursor-pointer bg-transparent border-none")}
          aria-label="Scroll to content"
        >
          <span className="text-[0.65rem] font-semibold tracking-[0.2em] uppercase">Explore</span>
          <ChevronDown className={clsx("w-4 h-4", styles.animateBounceGentle)} />
        </button>
      </div>
    </section>
  );
}
