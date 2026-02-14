import React, { useEffect, useRef, useState } from 'react';
import { BookOpen, GraduationCap, FlaskConical, Calendar } from 'lucide-react';
import { STATS } from '../../data/courseData';

const iconMap = {
  book: BookOpen,
  graduation: GraduationCap,
  flask: FlaskConical,
  calendar: Calendar,
};

export default function StatsBar() {
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
      { threshold: 0.3 }
    );
    if (sectionRef.current) observer.observe(sectionRef.current);
    return () => observer.disconnect();
  }, []);

  return (
    <section
      ref={sectionRef}
      className="relative py-10 md:py-14 bg-gradient-to-r from-blue-600 via-indigo-600 to-purple-700 dark:from-blue-900 dark:via-indigo-900 dark:to-purple-950 overflow-hidden"
    >
      {/* Subtle background pattern */}
      <div className="absolute inset-0 opacity-10" aria-hidden="true">
        <div className="absolute top-0 left-1/4 w-64 h-64 bg-white rounded-full filter blur-3xl" />
        <div className="absolute bottom-0 right-1/4 w-48 h-48 bg-white rounded-full filter blur-3xl" />
      </div>

      <div className="relative container mx-auto px-6 lg:px-8">
        <div className="grid grid-cols-2 md:grid-cols-4 gap-8 md:gap-12">
          {STATS.map((stat, index) => {
            const Icon = iconMap[stat.icon] || BookOpen;
            return (
              <div
                key={stat.label}
                className={`flex flex-col items-center gap-2 text-center transition-all duration-700 ${
                  isVisible
                    ? 'opacity-100 translate-y-0'
                    : 'opacity-0 translate-y-4'
                }`}
                style={{
                  transitionDelay: isVisible ? `${index * 100}ms` : '0ms',
                }}
              >
                <Icon className="w-6 h-6 text-blue-200" strokeWidth={1.5} />
                <span className="text-3xl md:text-4xl font-bold text-white">
                  {stat.value}
                </span>
                <span className="text-sm md:text-base text-blue-100 font-medium">
                  {stat.label}
                </span>
              </div>
            );
          })}
        </div>
      </div>
    </section>
  );
}
