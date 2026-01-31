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
    description: 'Understand the structure and goals of the Physical AI & Humanoid Robotics course',
    to: '/docs/course-overview',
    icon: '📖',
  },
  {
    title: 'Learning Objectives',
    description: 'Review what you will learn in this comprehensive textbook',
    to: '/docs/learning-objectives',
    icon: '🎯',
  },
  {
    title: 'Weekly Breakdown',
    description: 'Detailed schedule and topics for each week of study',
    to: '/docs/syllabus',
    icon: '📅',
  },
  {
    title: 'Assessments',
    description: 'Understand the evaluation methods and grading criteria',
    to: '/docs/assessments',
    icon: '📝',
  },
  {
    title: 'Hardware Requirements',
    description: 'Review the equipment and tools needed for practical exercises',
    to: '/docs/hardware-requirements',
    icon: '⚙️',
  },
];

function MetaItem({ item }: MetaItemComponentProps): React.JSX.Element {
  return (
    <div className="w-full md:w-1/2 lg:w-1/3 p-3">
      <Link
        to={item.to}
        className="block bg-white dark:bg-gray-800 rounded-lg p-4 hover:shadow-md transition-shadow duration-300 h-full border border-gray-200 dark:border-gray-700"
      >
        <div className="text-xl mb-2">{item.icon}</div>
        <Heading as="h3" className="text-md font-semibold text-gray-900 dark:text-white mb-2">
          {item.title}
        </Heading>
        <p className="text-gray-600 dark:text-gray-300 text-sm mb-2">{item.description}</p>
        <span className="text-blue-600 dark:text-blue-400 font-medium text-xs">Learn more →</span>
      </Link>
    </div>
  );
}

function CourseMetaSection({ className }: CourseMetaSectionProps = {}): React.JSX.Element {
  return (
    <section className={`py-12 ${className || ''}`}>
      <div className="container mx-auto px-4">
        <div className="mb-8">
          <Heading as="h2" className="text-2xl font-bold text-center text-gray-900 dark:text-white">
            Course Resources
          </Heading>
        </div>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
          {MetaItems.map((item, idx) => (
            <MetaItem key={idx} item={item} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default CourseMetaSection;