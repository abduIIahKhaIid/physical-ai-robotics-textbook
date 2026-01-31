import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  description: React.JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Foundations',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn the fundamentals of Physical AI - how artificial intelligence differs when operating
        in physical systems rather than purely digital environments.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Explore the principles of humanoid robotics, including kinematics, dynamics,
        control systems, and locomotion for bipedal robots.
      </>
    ),
  },
  {
    title: 'Advanced AI Integration',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Discover how advanced AI techniques are applied to humanoid systems for perception,
        learning, and interaction with the physical world.
      </>
    ),
  },
];

function Feature({Svg, title, description}: FeatureItem): React.JSX.Element {
  return (
    <div className="w-full md:w-1/3 p-4">
      <div className="flex flex-col items-center text-center">
        <div className="mb-4">
          <Svg className="w-16 h-16" role="img" />
        </div>
        <Heading as="h3" className="text-lg font-bold mb-2">{title}</Heading>
        <p className="text-gray-600 dark:text-gray-300 text-sm">{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className="py-12 bg-gray-50 dark:bg-gray-800">
      <div className="container mx-auto px-4">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}