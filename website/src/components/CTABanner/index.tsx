import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';

export default function CTABanner() {
  return (
    <section className="py-16 md:py-24 bg-gradient-to-b from-gray-50 to-gray-100 dark:from-gray-950 dark:to-gray-900/50">
      <div className="container mx-auto px-6 lg:px-8">
        <div className="relative overflow-hidden bg-gradient-to-r from-blue-600 to-indigo-700 dark:from-blue-900 dark:to-indigo-950 rounded-3xl shadow-2xl dark:shadow-indigo-900/50 p-12 text-center">

          <div className="relative z-10">
            <h2 className="text-3xl md:text-4xl font-bold text-white mb-4">
              Ready to Build Intelligent Robots?
            </h2>

            <p className="max-w-2xl mx-auto text-lg text-blue-100 dark:text-blue-200 mb-8">
              Dive into an open-source curriculum designed to take you from Physical AI foundations to advanced humanoid systems. Start with Module 1 and build your first robot controller.
            </p>

            <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
              <Link
                to="/docs/module-1/"
                className={clsx(
                  'inline-flex items-center justify-center px-8 py-4 text-lg font-semibold text-white bg-white/20 backdrop-blur-sm rounded-xl shadow-lg',
                  'transition-all duration-300 ease-in-out no-underline',
                  'hover:scale-105 hover:shadow-xl hover:bg-white/30 hover:text-white hover:no-underline'
                )}
              >
                Start Learning Now
              </Link>
              <Link
                to="/docs/syllabus"
                className={clsx(
                  'inline-flex items-center justify-center px-8 py-4 text-lg font-semibold text-white bg-black/30 dark:bg-black/50 backdrop-blur-sm rounded-xl shadow-lg dark:shadow-black/30',
                  'transition-all duration-300 ease-in-out no-underline',
                  'hover:scale-105 hover:shadow-xl hover:bg-black/40 dark:hover:bg-black/60 hover:text-white hover:no-underline'
                )}
              >
                View Full Syllabus
              </Link>
            </div>
          </div>

          {/* Abstract background shapes */}
          <div className="absolute top-0 left-0 w-32 h-32 bg-white/10 dark:bg-white/5 rounded-full -translate-x-1/2 -translate-y-1/2" aria-hidden="true" />
          <div className="absolute bottom-0 right-0 w-48 h-48 bg-white/10 dark:bg-white/5 rounded-full translate-x-1/2 translate-y-1/2" aria-hidden="true" />
        </div>
      </div>
    </section>
  );
}
