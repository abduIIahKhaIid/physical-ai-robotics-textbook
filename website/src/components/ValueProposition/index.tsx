import { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import {
  MousePointerClick, Code2, BookOpenCheck, FlaskConical,
} from 'lucide-react';
import { VALUE_PROPOSITIONS } from '../../data/courseData';

const accentStyles = {
  blue: {
    iconBg: 'bg-blue-100 dark:bg-blue-900/30',
    iconText: 'text-blue-600 dark:text-blue-400',
    hoverBorder: 'hover:border-blue-300 dark:hover:border-blue-700',
    glowShadow: 'hover:shadow-blue-100/50 dark:hover:shadow-blue-900/30',
    number: 'text-blue-200 dark:text-blue-800/40',
  },
  purple: {
    iconBg: 'bg-purple-100 dark:bg-purple-900/30',
    iconText: 'text-purple-600 dark:text-purple-400',
    hoverBorder: 'hover:border-purple-300 dark:hover:border-purple-700',
    glowShadow: 'hover:shadow-purple-100/50 dark:hover:shadow-purple-900/30',
    number: 'text-purple-200 dark:text-purple-800/40',
  },
  cyan: {
    iconBg: 'bg-cyan-100 dark:bg-cyan-900/30',
    iconText: 'text-cyan-600 dark:text-cyan-400',
    hoverBorder: 'hover:border-cyan-300 dark:hover:border-cyan-700',
    glowShadow: 'hover:shadow-cyan-100/50 dark:hover:shadow-cyan-900/30',
    number: 'text-cyan-200 dark:text-cyan-800/40',
  },
  indigo: {
    iconBg: 'bg-indigo-100 dark:bg-indigo-900/30',
    iconText: 'text-indigo-600 dark:text-indigo-400',
    hoverBorder: 'hover:border-indigo-300 dark:hover:border-indigo-700',
    glowShadow: 'hover:shadow-indigo-100/50 dark:hover:shadow-indigo-900/30',
    number: 'text-indigo-200 dark:text-indigo-800/40',
  },
};

const iconMap = {
  'pointer': MousePointerClick,
  'code': Code2,
  'book-check': BookOpenCheck,
  'flask': FlaskConical,
};

export default function ValueProposition() {
  const sectionRef = useRef(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.disconnect();
        }
      },
      { threshold: 0.15 }
    );
    if (sectionRef.current) observer.observe(sectionRef.current);
    return () => observer.disconnect();
  }, []);

  return (
    <section id="features" className="relative py-20 md:py-28 bg-gray-50 dark:bg-gray-950 overflow-hidden">
      {/* Subtle background accent */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none" aria-hidden="true">
        <div className="absolute -top-24 -right-24 w-96 h-96 bg-blue-400/5 dark:bg-blue-400/[0.03] rounded-full filter blur-3xl" />
        <div className="absolute -bottom-24 -left-24 w-96 h-96 bg-purple-400/5 dark:bg-purple-400/[0.03] rounded-full filter blur-3xl" />
      </div>

      <div ref={sectionRef} className="relative container mx-auto px-6 lg:px-8">
        {/* Section header */}
        <div className={clsx(
          'max-w-3xl mx-auto text-center mb-16 md:mb-20 transition-all duration-700',
          isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-6'
        )}>
          <p className="text-sm font-semibold uppercase tracking-[0.2em] text-blue-600 dark:text-blue-400 mb-3">
            Why This Textbook
          </p>
          <h2 className="text-3xl sm:text-4xl md:text-5xl font-extrabold text-gray-900 dark:text-white mb-5 tracking-tight">
            A Modern Approach to Robotics Education
          </h2>
          <p className="text-lg md:text-xl text-gray-600 dark:text-gray-400 leading-relaxed max-w-2xl mx-auto">
            Beyond traditional textbooks — an interactive, community-driven platform designed to take you from first principles to real-world deployment.
          </p>
        </div>

        {/* Cards grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6 lg:gap-8">
          {VALUE_PROPOSITIONS.map((prop, index) => {
            const styles = accentStyles[prop.accent];
            const Icon = iconMap[prop.icon] || MousePointerClick;
            return (
              <div
                key={index}
                className={clsx(
                  'group relative p-8 rounded-2xl transition-all duration-300',
                  'bg-white dark:bg-gray-900',
                  'border border-gray-200/80 dark:border-gray-800',
                  styles.hoverBorder,
                  'hover:shadow-xl dark:hover:shadow-lg',
                  styles.glowShadow,
                  'hover:-translate-y-1.5',
                  isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-8',
                )}
                style={{
                  transition: 'all 0.5s cubic-bezier(0.4, 0, 0.2, 1)',
                  transitionDelay: isVisible ? `${150 + index * 100}ms` : '0ms',
                }}
              >
                {/* Background number */}
                <span className={clsx(
                  'absolute top-4 right-5 text-6xl font-black select-none pointer-events-none',
                  styles.number
                )} aria-hidden="true">
                  {String(index + 1).padStart(2, '0')}
                </span>

                {/* Icon */}
                <div className={clsx(
                  'relative inline-flex items-center justify-center w-14 h-14 rounded-xl mb-5 transition-transform duration-300 group-hover:scale-110',
                  styles.iconBg
                )}>
                  <Icon className={clsx('w-7 h-7', styles.iconText)} strokeWidth={1.5} />
                </div>

                {/* Content */}
                <h3 className="relative text-xl font-bold text-gray-900 dark:text-white mb-3">
                  {prop.title}
                </h3>
                <p className="relative text-base text-gray-600 dark:text-gray-400 leading-relaxed">
                  {prop.description}
                </p>
              </div>
            );
          })}
        </div>
      </div>
    </section>
  );
}
