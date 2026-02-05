import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';

type MetaItemProps = {
  title: string;
  description: string;
  to: string;
  icon: string;
};

type MetaItemComponentProps = {
  item: MetaItemProps;
};

type CourseMetaSectionProps = {
  className?: string;
};

const MetaItems: MetaItemProps[] = [
  {
    title: 'Course Overview',
    description: 'Explore the complete structure, goals, and learning pathway of the Physical AI & Humanoid Robotics program',
    to: '/docs/course-overview',
    icon: '📚',
  },
  {
    title: 'Learning Objectives',
    description: 'Discover the specific skills, knowledge, and competencies you will develop throughout the course',
    to: '/docs/learning-objectives',
    icon: '🎯',
  },
  {
    title: 'Weekly Breakdown',
    description: 'Follow the detailed 13-week schedule with topics, assignments, and milestones for each module',
    to: '/docs/syllabus',
    icon: '📅',
  },
  {
    title: 'Assessments',
    description: 'Understand the diverse evaluation methods, grading criteria, and project requirements',
    to: '/docs/assessments',
    icon: '✅',
  },
  {
    title: 'Hardware Requirements',
    description: 'Get detailed specifications for computing, robotics kits, and tools needed for hands-on learning',
    to: '/docs/hardware-requirements',
    icon: '💻',
  },
];

function MetaItem({ item }: MetaItemComponentProps): React.JSX.Element {
  return (
    <Link
      to={item.to}
      className="block bg-white dark:bg-slate-800/60 backdrop-blur-sm rounded-2xl p-6 hover:shadow-2xl transition-all duration-500 h-full border border-slate-200 dark:border-slate-700/50 hover:border-blue-500 dark:hover:border-blue-400 hover:bg-white/95 dark:hover:bg-slate-700/80 group relative overflow-hidden"
    >
      {/* Animated background effect */}
      <div className="absolute inset-0 bg-gradient-to-br from-blue-50/30 to-purple-50/30 dark:from-blue-900/10 dark:to-purple-900/10 opacity-0 group-hover:opacity-100 transition-opacity duration-500"></div>

      {/* Content with z-index to stay above background */}
      <div className="relative z-10">
        <div className="flex items-start gap-5">
          <div className="flex-shrink-0 w-14 h-14 rounded-xl bg-gradient-to-r from-blue-500 to-purple-500 flex items-center justify-center text-white text-2xl group-hover:scale-110 transition-transform duration-500 shadow-lg group-hover:shadow-xl">
            {item.icon}
          </div>
          <div className="flex-1 min-w-0">
            <Heading as="h3" className="text-xl font-bold text-slate-900 dark:text-white mb-3 group-hover:text-blue-600 dark:group-hover:text-blue-400 transition-colors duration-300">
              {item.title}
            </Heading>
            <p className="text-slate-600 dark:text-slate-300 text-base mb-5 leading-relaxed">
              {item.description}
            </p>
            <div className="flex items-center gap-2 text-blue-600 dark:text-blue-400 font-semibold text-sm group-hover:gap-3 transition-all duration-300">
              <span>Explore</span>
              <svg
                className="w-4 h-4 transition-transform duration-300 group-hover:translate-x-1"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            </div>
          </div>
        </div>
      </div>
    </Link>
  );
}

function CourseMetaSection({ className }: CourseMetaSectionProps = {}): React.JSX.Element {
  return (
    <section className={`py-24 bg-gradient-to-br from-slate-50 to-white dark:from-slate-900 dark:to-slate-950 ${className || ''}`}>
      <div className="container mx-auto px-4">
        <div className="max-w-4xl mx-auto text-center mb-16">
          <div className="inline-flex items-center justify-center w-20 h-20 bg-gradient-to-r from-blue-600 to-purple-600 rounded-2xl mb-6 shadow-2xl mx-auto">
            <svg className="w-10 h-10 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
            </svg>
          </div>
          <Heading as="h2" className="text-4xl md:text-5xl font-bold text-center text-slate-900 dark:text-white mb-4">
            Course Essentials
          </Heading>
          <p className="text-lg md:text-xl text-slate-600 dark:text-slate-300 max-w-2xl mx-auto leading-relaxed">
            Explore the essential components of your Physical AI & Humanoid Robotics learning journey
          </p>
        </div>

        {/* Enhanced grid layout with better spacing and visual hierarchy */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
          {MetaItems.map((item, idx) => (
            <MetaItem key={idx} item={item} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default CourseMetaSection;