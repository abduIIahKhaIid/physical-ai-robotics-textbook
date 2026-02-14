import { useEffect, useRef } from 'react';

// Performance optimized particle system
export const useParticleAnimation = (
  canvasRef: React.RefObject<HTMLCanvasElement>,
  isDarkTheme: boolean,
  enabled: boolean = true
) => {
  const animationFrameId = useRef<number | null>(null);
  const particlesRef = useRef<any[]>([]);
  const mouseRef = useRef<{ x: number | null; y: number | null; radius: number }>({ 
    x: null, 
    y: null, 
    radius: 120 
  });

  useEffect(() => {
    if (!enabled || !canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Particle class definition
    class Particle {
      x: number;
      y: number;
      size: number;
      baseX: number;
      baseY: number;
      density: number;
      vx: number;
      vy: number;
      originalSize: number;

      constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
        this.originalSize = Math.random() * 1.5 + 0.5;
        this.size = this.originalSize;
        this.baseX = x;
        this.baseY = y;
        this.density = Math.random() * 30 + 1;
        this.vx = (Math.random() - 0.5) * 0.3;
        this.vy = (Math.random() - 0.5) * 0.3;
      }

      draw(ctx: CanvasRenderingContext2D, isDarkTheme: boolean) {
        // Optimized drawing with cached gradients
        const gradient = ctx.createRadialGradient(
          this.x, this.y, 0,
          this.x, this.y, this.size * 3
        );

        if (isDarkTheme) {
          gradient.addColorStop(0, `rgba(147, 197, 253, ${0.8 + this.size * 0.2})`);
          gradient.addColorStop(1, `rgba(147, 197, 253, 0)`);
        } else {
          gradient.addColorStop(0, `rgba(59, 130, 246, ${0.7 + this.size * 0.15})`);
          gradient.addColorStop(1, `rgba(59, 130, 246, 0)`);
        }

        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.size * 3, 0, Math.PI * 2);
        ctx.fill();

        // Core particle
        ctx.fillStyle = isDarkTheme
          ? `rgba(147, 197, 253, ${0.9 + this.size * 0.2})`
          : `rgba(59, 130, 246, ${0.85 + this.size * 0.15})`;
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
        ctx.closePath();
        ctx.fill();
      }

      update(mouse: typeof mouseRef.current) {
        // Ambient drift
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

            // Increase size when near mouse
            this.size = Math.min(this.originalSize * 2, this.originalSize + 1);
          } else {
            this.x += (this.baseX - this.x) * 0.05;
            this.y += (this.baseY - this.y) * 0.05;
            // Return to original size
            this.size += (this.originalSize - this.size) * 0.1;
          }
        } else {
          this.x += (this.baseX - this.x) * 0.05;
          this.y += (this.baseY - this.y) * 0.05;
          // Return to original size
          this.size += (this.originalSize - this.size) * 0.1;
        }
      }
    }

    const resizeCanvas = () => {
      if (canvas.parentElement) {
        canvas.width = window.innerWidth;
        canvas.height = canvas.parentElement.offsetHeight || window.innerHeight;
      }
    };

    const initParticles = () => {
      const count = Math.min((canvas.width * canvas.height) / 8000, 150); // Reduced particle count for performance
      particlesRef.current = [];
      for (let i = 0; i < count; i++) {
        particlesRef.current.push(new Particle(
          Math.random() * canvas.width * 0.8 + canvas.width * 0.1,
          Math.random() * canvas.height * 0.8 + canvas.height * 0.1
        ));
      }
    };

    const connectParticles = () => {
      for (let a = 0; a < particlesRef.current.length; a++) {
        for (let b = a + 1; b < particlesRef.current.length; b++) {
          const dx = particlesRef.current[a].x - particlesRef.current[b].x;
          const dy = particlesRef.current[a].y - particlesRef.current[b].y;
          const distance = Math.sqrt(dx * dx + dy * dy);
          if (distance < 130) {
            const opacity = (1 - distance / 130) * 0.35;
            const gradient = ctx.createLinearGradient(
              particlesRef.current[a].x, particlesRef.current[a].y,
              particlesRef.current[b].x, particlesRef.current[b].y
            );

            if (isDarkTheme) {
              gradient.addColorStop(0, `rgba(147, 197, 253, ${opacity})`);
              gradient.addColorStop(1, `rgba(99, 102, 241, ${opacity * 0.5})`);
            } else {
              gradient.addColorStop(0, `rgba(59, 130, 246, ${opacity * 0.7})`);
              gradient.addColorStop(1, `rgba(99, 102, 241, ${opacity * 0.35})`);
            }

            ctx.strokeStyle = gradient;
            ctx.lineWidth = 0.6 + (1 - distance / 130) * 0.4;
            ctx.beginPath();
            ctx.moveTo(particlesRef.current[a].x, particlesRef.current[a].y);
            ctx.lineTo(particlesRef.current[b].x, particlesRef.current[b].y);
            ctx.stroke();
          }
        }
      }
    };

    const animate = () => {
      if (!ctx) return;
      
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Batch operations for better performance
      particlesRef.current.forEach(p => {
        p.update(mouseRef.current);
        p.draw(ctx, isDarkTheme);
      });
      
      connectParticles();
      
      animationFrameId.current = requestAnimationFrame(animate);
    };

    const handleMouseMove = (e: MouseEvent) => {
      const rect = canvas.getBoundingClientRect();
      mouseRef.current.x = e.clientX - rect.left;
      mouseRef.current.y = e.clientY - rect.top;
    };

    const handleMouseLeave = () => {
      mouseRef.current.x = null;
      mouseRef.current.y = null;
    };

    const handleResize = () => {
      resizeCanvas();
      initParticles();
    };

    // Initialize
    resizeCanvas();
    initParticles();
    animate();

    // Event listeners
    window.addEventListener('resize', handleResize);
    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('mouseleave', handleMouseLeave);

    // Cleanup
    return () => {
      if (animationFrameId.current) {
        cancelAnimationFrame(animationFrameId.current);
      }
      window.removeEventListener('resize', handleResize);
      canvas.removeEventListener('mousemove', handleMouseMove);
      canvas.removeEventListener('mouseleave', handleMouseLeave);
    };
  }, [isDarkTheme, enabled]);

  return null;
};