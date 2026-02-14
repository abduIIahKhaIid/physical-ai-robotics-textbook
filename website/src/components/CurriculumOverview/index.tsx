import React, { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { ArrowRight, Clock, BookOpen, FlaskConical, Trophy } from 'lucide-react';
import { MODULES } from '../../data/courseData';

const colorStyles = {
  blue: {
    badge: 'bg-blue-100 dark:bg-blue-900/30 text-blue-600 dark:text-blue-400',
    bar: 'bg-blue-500',
    link: 'text-blue-600 dark:text-blue-400',
    border: 'hover:border-blue-300 dark:hover:border-blue-700',
    dot: 'bg-blue-500',
    line: 'from-blue-500',
  },
  purple: {
    badge: 'bg-purple-100 dark:bg-purple-900/30 text-purple-600 dark:text-purple-400',
    bar: 'bg-purple-500',
    link: 'text-purple-600 dark:text-purple-400',
    border: 'hover:border-purple-300 dark:hover:border-purple-700',
    dot: 'bg-purple-500',
    line: 'from-purple-500',
  },
  cyan: {
    badge: 'bg-cyan-100 dark:bg-cyan-900/30 text-cyan-600 dark:text-cyan-400',
    bar: 'bg-cyan-500',
    link: 'text-cyan-600 dark:text-cyan-400',
    border: 'hover:border-cyan-300 dark:hover:border-cyan-700',
    dot: 'bg-cyan-500',
    line: 'from-cyan-500',
  },
  indigo: {
    badge: 'bg-indigo-100 dark:bg-indigo-900/30 text-indigo-600 dark:text-indigo-400',
    bar: 'bg-indigo-500',
    link: 'text-indigo-600 dark:text-indigo-400',
    border: 'hover:border-indigo-300 dark:hover:border-indigo-700',
    dot: 'bg-indigo-500',
    line: 'from-indigo-500',
  },
};

export default function CurriculumOverview() {
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
      { threshold: 0.1 }
    );
    if (sectionRef.current) observer.observe(sectionRef.current);
    return () => observer.disconnect();
  }, []);

  return (
    <section id="curriculum" className="py-20 md:py-28 bg-gradient-to-b from-gray-100 to-gray-50 dark:from-gray-900 dark:to-gray-950">
      <div ref={sectionRef} className="container mx-auto px-6 lg:px-8">
        <div className={clsx(
          'max-w-3xl mx-auto text-center mb-16 transition-all duration-700',
          isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-6'
        )}>
          <p className="text-sm font-semibold uppercase tracking-[0.2em] text-blue-600 dark:text-blue-400 mb-3">
            Curriculum
          </p>
          <h2 className="text-3xl md:text-4xl font-bold text-gray-900 dark:text-white mb-4">
            Comprehensive Learning Path
          </h2>
          <p className="text-lg text-gray-600 dark:text-gray-400">
            From Physical AI foundations to advanced humanoid systems — a 13-week journey building expertise step-by-step.
          </p>
        </div>

        <div className="relative max-w-6xl mx-auto">
          {/* Timeline connector (desktop only) */}
          <div className="hidden md:block absolute left-1/2 top-0 bottom-0 w-px bg-gradient-to-b from-blue-300 via-purple-300 via-cyan-300 to-indigo-300 dark:from-blue-700 dark:via-purple-700 dark:via-cyan-700 dark:to-indigo-700" aria-hidden="true" />

          <div className="space-y-8 md:space-y-12">
            {MODULES.map((module, index) => {
              const colors = colorStyles[module.color] || colorStyles.blue;
              const isLeft = index % 2 === 0;

              return (
                <div
                  key={module.id}
                  className={clsx(
                    'relative md:grid md:grid-cols-2 md:gap-8 items-center transition-all duration-700',
                    isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-8',
                  )}
                  style={{
                    transitionDelay: isVisible ? `${200 + index * 150}ms` : '0ms',
                  }}
                >
                  {/* Timeline dot (desktop only) */}
                  <div className={clsx(
                    'hidden md:flex absolute left-1/2 -translate-x-1/2 w-10 h-10 rounded-full items-center justify-center z-10 border-4 border-white dark:border-gray-950',
                    colors.dot
                  )}>
                    <span className="text-white font-bold text-sm">{module.id}</span>
                  </div>

                  {/* Card */}
                  <div className={clsx(
                    'group relative p-6 lg:p-8 rounded-2xl transition-all duration-300',
                    'bg-white dark:bg-gray-900',
                    'border border-gray-200/80 dark:border-gray-800',
                    colors.border,
                    'hover:shadow-xl dark:hover:shadow-lg',
                    'hover:-translate-y-1',
                    'overflow-hidden',
                    isLeft ? 'md:col-start-1' : 'md:col-start-2',
                  )}>
                    {/* Top accent bar */}
                    <div className={clsx('absolute top-0 left-0 w-full h-1', colors.bar)} />

                    <div className="relative z-10">
                      {/* Module header */}
                      <div className="flex items-center mb-4">
                        <div className={clsx(
                          'flex items-center justify-center w-10 h-10 rounded-full mr-4 md:hidden',
                          colors.badge
                        )}>
                          <span className="font-bold text-lg">{module.id}</span>
                        </div>
                        <h3 className="text-xl font-bold text-gray-900 dark:text-white">
                          {module.title}
                        </h3>
                      </div>

                      <p className="text-gray-600 dark:text-gray-400 mb-4 md:pl-0">
                        {module.description}
                      </p>

                      {/* Module metadata */}
                      <div className="flex flex-wrap items-center gap-4 text-sm text-gray-500 dark:text-gray-500 mb-4">
                        <span className="inline-flex items-center gap-1.5">
                          <Clock className="w-4 h-4" />
                          {module.duration}
                        </span>
                        <span className="inline-flex items-center gap-1.5">
                          <BookOpen className="w-4 h-4" />
                          {module.lessons} lessons
                        </span>
                        <span className="inline-flex items-center gap-1.5">
                          <FlaskConical className="w-4 h-4" />
                          {module.labs} labs
                        </span>
                        {module.capstone && (
                          <span className="inline-flex items-center gap-1.5">
                            <Trophy className="w-4 h-4" />
                            Capstone
                          </span>
                        )}
                      </div>

                      {/* Topics list */}
                      {module.topics && (
                        <div className="mb-4">
                          <p className="text-xs font-semibold uppercase tracking-wider text-gray-400 dark:text-gray-500 mb-2">
                            Topics
                          </p>
                          <ul className="space-y-1.5">
                            {module.topics.map((topic, i) => (
                              <li key={i} className="flex items-start gap-2 text-sm text-gray-600 dark:text-gray-400">
                                <span className={clsx('mt-1.5 w-1.5 h-1.5 rounded-full flex-shrink-0', colors.dot)} />
                                {topic}
                              </li>
                            ))}
                          </ul>
                        </div>
                      )}

                      {/* Explore link */}
                      <Link
                        to={module.link}
                        className={clsx(
                          'font-semibold inline-flex items-center gap-1 no-underline',
                          colors.link,
                          'hover:underline group-hover:gap-2 transition-all'
                        )}
                      >
                        Explore Module
                        <ArrowRight className="w-4 h-4 transition-transform group-hover:translate-x-1" />
                      </Link>
                    </div>
                  </div>

                  {/* Spacer for alternating layout */}
                  {isLeft ? <div className="hidden md:block" /> : null}
                </div>
              );
            })}
          </div>
        </div>
      </div>
    </section>
  );
}
