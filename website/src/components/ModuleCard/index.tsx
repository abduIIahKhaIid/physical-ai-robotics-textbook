import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';

type ModuleCardProps = {
  id?: string;
  title: string;
  description: string;
  path: string;
};

function ModuleCard({ id, title, description, path }: ModuleCardProps): React.JSX.Element {
  return (
    <div className="w-full">
      <div className="group relative bg-white dark:bg-slate-800/50 backdrop-blur-sm rounded-2xl shadow-lg hover:shadow-2xl transition-all duration-500 h-full flex flex-col border border-slate-200/50 dark:border-slate-700/50 overflow-hidden">
        {/* Hover effect background */}
        <div className="absolute inset-0 bg-gradient-to-br from-blue-50/50 to-purple-50/50 dark:from-blue-900/10 dark:to-purple-900/10 opacity-0 group-hover:opacity-100 transition-opacity duration-500"></div>

        <div className="relative p-8 flex-grow">
          <div className="mb-6">
            <div className="w-14 h-14 bg-gradient-to-r from-blue-600 to-purple-600 rounded-xl flex items-center justify-center mb-4 group-hover:scale-110 transition-transform duration-300">
              <span className="text-white font-bold text-lg">
                {title.split(' ')[1]?.replace(':', '') || '1'}
              </span>
            </div>
            <h3 className="text-xl font-bold text-slate-900 dark:text-white mb-3 group-hover:text-blue-600 dark:group-hover:text-blue-400 transition-colors duration-300">
              {title}
            </h3>
            <p className="text-slate-600 dark:text-slate-300 text-base leading-relaxed">
              {description}
            </p>
          </div>
        </div>

        <div className="relative p-8 pt-0">
          <Link
            to={path}
            className="group/btn relative inline-block bg-gradient-to-r from-blue-600 to-purple-600 hover:from-blue-700 hover:to-purple-700 text-white font-semibold py-3 px-6 rounded-xl transition-all duration-300 text-center w-full shadow-lg hover:shadow-xl group-hover:-translate-y-0.5"
          >
            <span className="relative z-10 flex items-center justify-center gap-2">
              <span>Explore Module</span>
              <svg className="w-4 h-4 transition-transform duration-300 group-hover/btn:translate-x-1" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            </span>
            <div className="absolute inset-0 bg-gradient-to-r from-purple-600 to-blue-600 rounded-xl opacity-0 group-hover/btn:opacity-100 transition-opacity duration-300"></div>
          </Link>
        </div>
      </div>
    </div>
  );
}

export default ModuleCard;