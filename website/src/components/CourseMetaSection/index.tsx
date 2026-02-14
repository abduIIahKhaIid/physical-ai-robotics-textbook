import React, { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { Calendar, Target, CheckSquare, HardDrive, ArrowRight } from 'lucide-react';
import { RESOURCES } from '../../data/courseData';

const iconMap = {
  calendar: Calendar,
  target: Target,
  checkmark: CheckSquare,
  harddrive: HardDrive,
};

const colorStyles = {
  blue: 'bg-blue-100 dark:bg-blue-900/30 text-blue-600 dark:text-blue-400',
  purple: 'bg-purple-100 dark:bg-purple-900/30 text-purple-600 dark:text-purple-400',
  cyan: 'bg-cyan-100 dark:bg-cyan-900/30 text-cyan-600 dark:text-cyan-400',
  indigo: 'bg-indigo-100 dark:bg-indigo-900/30 text-indigo-600 dark:text-indigo-400',
};

export default function CourseMetaSection() {
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
    <section id="resources" className="py-16 md:py-24 bg-gradient-to-b from-gray-50 to-gray-100 dark:from-gray-950 dark:to-gray-900">
      <div ref={sectionRef} className="container mx-auto px-6 lg:px-8">
        <div className={clsx(
          'max-w-3xl mx-auto text-center mb-16 transition-all duration-700',
          isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-6'
        )}>
          <p className="text-sm font-semibold uppercase tracking-[0.2em] text-blue-600 dark:text-blue-400 mb-3">
            Getting Started
          </p>
          <h2 className="text-3xl md:text-4xl font-bold text-gray-900 dark:text-white mb-4">
            Course Resources
          </h2>
          <p className="text-lg text-gray-600 dark:text-gray-400">
            Everything you need to begin your journey in Physical AI and Robotics.
          </p>
        </div>

        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6">
          {RESOURCES.map((item, index) => {
            const Icon = iconMap[item.icon] || Calendar;
            return (
              <Link
                key={item.title}
                to={item.to}
                className={clsx(
                  'group block p-6 bg-white dark:bg-gray-900 rounded-2xl border border-gray-200/80 dark:border-gray-800',
                  'transition-all duration-300 hover:shadow-xl dark:hover:shadow-lg hover:-translate-y-1.5',
                  'overflow-hidden no-underline',
                  isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-8',
                )}
                style={{
                  transitionDelay: isVisible ? `${100 + index * 100}ms` : '0ms',
                }}
              >
                <div className={clsx(
                  'inline-flex items-center justify-center w-14 h-14 rounded-xl mb-4',
                  colorStyles[item.color] || colorStyles.blue
                )}>
                  <Icon className="h-6 w-6" strokeWidth={1.5} />
                </div>
                <h3 className="text-lg font-bold text-gray-900 dark:text-white mb-2">
                  {item.title}
                </h3>
                <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                  {item.description}
                </p>
                <span className="inline-flex items-center gap-1 text-sm font-medium text-blue-600 dark:text-blue-400 group-hover:gap-2 transition-all">
                  View
                  <ArrowRight className="w-3.5 h-3.5 transition-transform group-hover:translate-x-1" />
                </span>
              </Link>
            );
          })}
        </div>
      </div>
    </section>
  );
}
