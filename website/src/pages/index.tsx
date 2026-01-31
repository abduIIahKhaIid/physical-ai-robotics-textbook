import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ModuleCard from '@site/src/components/ModuleCard';
import CourseMetaSection from '@site/src/components/CourseMetaSection';
import ChatbotTeaser from '@site/src/components/ChatbotTeaser';

// Define types for module data
type ModuleData = {
  id: string;
  title: string;
  description: string;
  path: string;
};

function HomepageHeader(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  const [scrollY, setScrollY] = useState(0);

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <header
      className={clsx(
        "relative overflow-hidden bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white",
        "transition-all duration-300"
      )}
      style={{ transform: `translateY(${scrollY * 0.1}px)` }}
    >
      {/* Animated background elements */}
      <div className="absolute inset-0 overflow-hidden">
        <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-purple-500/20 rounded-full blur-3xl animate-pulse"></div>
        <div className="absolute bottom-1/4 right-1/4 w-80 h-80 bg-blue-500/20 rounded-full blur-3xl animate-pulse delay-1000"></div>
        <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-96 h-96 bg-indigo-500/20 rounded-full blur-3xl animate-pulse delay-500"></div>
      </div>

      {/* Grid pattern overlay */}
      <div
        className="absolute inset-0 opacity-10"
        style={{
          backgroundImage: `radial-gradient(circle at 1px 1px, white 1px, transparent 0)`,
          backgroundSize: '50px 50px'
        }}
      ></div>

      <div className="relative container mx-auto px-4 py-28">
        <div className="max-w-5xl mx-auto text-center">
          <div className="mb-8">
            <div className="inline-flex items-center justify-center w-20 h-20 bg-white/10 backdrop-blur-sm rounded-2xl mb-6 border border-white/20">
              <svg className="w-10 h-10 text-blue-300" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
            </div>
          </div>

          <h1 className="text-5xl md:text-7xl lg:text-8xl font-bold mb-6 hero__title bg-clip-text text-transparent bg-gradient-to-r from-blue-300 via-purple-300 to-cyan-300 leading-tight">
            {siteConfig.title}
          </h1>

          <p className="text-xl md:text-3xl mb-12 text-blue-100/90 hero__subtitle max-w-4xl mx-auto leading-relaxed">
            {siteConfig.tagline}
          </p>

          <div className="flex flex-col sm:flex-row gap-6 justify-center items-center mb-16">
            <Link
              className={clsx(
                "group relative px-8 py-4 bg-gradient-to-r from-blue-600 to-purple-600 text-white font-bold text-lg rounded-xl shadow-lg",
                "hover:shadow-2xl transform hover:-translate-y-1 transition-all duration-300",
                "w-full sm:w-auto text-center overflow-hidden",
                "before:absolute before:inset-0 before:bg-white/20 before:opacity-0 before:transition-opacity before:duration-300",
                "before:hover:opacity-100"
              )}
              to="/docs/module-1/">
              <span className="relative z-10 flex items-center justify-center gap-3">
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
                <span>Start Learning</span>
              </span>
            </Link>

            <Link
              className={clsx(
                "group relative px-8 py-4 bg-white/10 backdrop-blur-sm text-white font-medium text-lg rounded-xl border border-white/20",
                "hover:bg-white/20 transition-all duration-300",
                "w-full sm:w-auto text-center overflow-hidden",
                "before:absolute before:inset-0 before:bg-white/10 before:opacity-0 before:transition-opacity before:duration-300",
                "before:hover:opacity-100"
              )}
              to="/docs/intro">
              <span className="relative z-10 flex items-center justify-center gap-3">
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                </svg>
                <span>View Syllabus</span>
              </span>
            </Link>
          </div>

          <div className="courseSummary max-w-3xl mx-auto">
            <p className="text-lg text-blue-100/80 leading-relaxed">
              This comprehensive textbook explores the intersection of Artificial Intelligence and Robotics,
              focusing on how intelligent systems can interact with and operate in the physical world.
              From fundamental concepts of Physical AI to advanced humanoid robotics, you'll gain deep
              insights into the future of embodied intelligence.
            </p>
          </div>
        </div>
      </div>

      {/* Wave divider - positioned to not overlap */}
      <div className="absolute bottom-0 left-0 w-full overflow-hidden z-10">
        <svg className="relative block w-full h-16 md:h-24 text-slate-50 dark:text-slate-900" preserveAspectRatio="none" viewBox="0 0 1200 120" xmlns="http://www.w3.org/2000/svg">
          <path d="M0 0v46.29c47.79 22.2 103.59 32.17 158 28 70.36-5.37 136.33-33.31 206.8-37.5 73.84-4.36 147.54 16.88 218.2 35.26 69.27 18 138.3 24.88 209.4 13.08 36.15-6 69.85-17.84 104.45-29.34C989.49 25 1113-14.29 1200 52.47V0z" fill="currentColor"></path>
        </svg>
      </div>
    </header>
  );
}

// Get module data
const moduleData: ModuleData[] = [
  {
    id: 'module-1',
    title: 'Module 1: Foundations of Physical AI',
    description: 'Explore the fundamental concepts of Physical AI and how intelligence differs when operating in physical systems.',
    path: '/docs/module-1/'
  },
  {
    id: 'module-2',
    title: 'Module 2: Kinematics and Dynamics',
    description: 'Understand the mathematical foundations of robot movement and the physics of robotic systems.',
    path: '/docs/module-2/'
  },
  {
    id: 'module-3',
    title: 'Module 3: Control Systems and Locomotion',
    description: 'Learn about feedback control, stability, and how robots achieve controlled movement.',
    path: '/docs/module-3/'
  },
  {
    id: 'module-4',
    title: 'Module 4: Perception and Machine Learning',
    description: 'Discover how robots perceive their environment and learn from sensory input.',
    path: '/docs/module-4/'
  }
];

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Educational textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main className={clsx(isVisible && "animate-in fade-in duration-1000")}>
        {/* Add spacing to account for wave divider */}
        <div className="h-16 md:h-24"></div>

        {/* Module Cards Section - adjusted padding to account for wave divider */}
        <section className="pt-16 pb-24 modulesSection bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
          <div className="container mx-auto px-4">
            <div className="max-w-4xl mx-auto text-center mb-20 -mt-16"> {/* Negative margin to account for wave */}
              <div className="inline-flex items-center justify-center w-20 h-20 bg-gradient-to-r from-blue-600 to-purple-600 rounded-2xl mb-8 shadow-lg">
                <svg className="w-10 h-10 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.746.477 3.332.977 4.5 1.753v13C19.832 18.477 18.246 18 16.5 18c-1.746.477-3.332.977-4.5 1.753" />
                </svg>
              </div>
              <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-slate-900 dark:text-white mb-6 sectionTitle">
                Course Modules
              </h2>
              <p className="text-lg md:text-xl text-slate-600 dark:text-slate-300 sectionSubtitle max-w-2xl mx-auto leading-relaxed">
                Structured learning path covering all aspects of Physical AI and Humanoid Robotics
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
              {moduleData.map((module, index) => (
                <div
                  key={module.id}
                  className={clsx(
                    "transform transition-all duration-700 ease-out",
                    isVisible ? "translate-y-0 opacity-100" : "translate-y-10 opacity-0",
                    `delay-${index * 100}`
                  )}
                  style={{ transitionDelay: `${index * 100}ms` }}
                >
                  <ModuleCard
                    id={module.id}
                    title={module.title}
                    description={module.description}
                    path={module.path}
                  />
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* Stats Section */}
        <section className="py-20 bg-gradient-to-r from-blue-50 via-purple-50 to-indigo-50 dark:from-slate-800 dark:to-slate-900">
          <div className="container mx-auto px-4">
            <div className="max-w-5xl mx-auto">
              <div className="grid grid-cols-2 md:grid-cols-4 gap-8 text-center">
                {[
                  { value: '4', label: 'Modules', icon: '📚' },
                  { value: '12', label: 'Weeks', icon: '📅' },
                  { value: '50+', label: 'Lessons', icon: '✍️' },
                  { value: '∞', label: 'Possibilities', icon: '✨' }
                ].map((stat, index) => (
                  <div
                    key={index}
                    className="group p-8 bg-white/50 dark:bg-slate-800/50 backdrop-blur-sm rounded-2xl border border-white/20 dark:border-slate-700/50 hover:bg-white/70 dark:hover:bg-slate-800/70 transition-all duration-300"
                  >
                    <div className="text-4xl md:text-5xl font-bold text-blue-600 dark:text-blue-400 mb-4 group-hover:scale-110 transition-transform duration-300">
                      {stat.value}
                    </div>
                    <div className="text-3xl mb-2">{stat.icon}</div>
                    <div className="text-slate-700 dark:text-slate-300 font-medium text-lg">
                      {stat.label}
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </section>

        {/* Features Section */}
        <section className="py-24 bg-white dark:bg-slate-900">
          <div className="container mx-auto px-4">
            <div className="max-w-6xl mx-auto">
              <div className="text-center mb-20">
                <h2 className="text-4xl md:text-5xl font-bold text-slate-900 dark:text-white mb-6">
                  Why Learn Physical AI & Robotics?
                </h2>
                <p className="text-lg text-slate-600 dark:text-slate-300 max-w-3xl mx-auto">
                  Discover the cutting-edge intersection of artificial intelligence and physical systems
                </p>
              </div>

              <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
                {[
                  {
                    title: 'Real-World Applications',
                    description: 'Learn how AI operates in physical environments with tangible outcomes',
                    icon: '🌍'
                  },
                  {
                    title: 'Advanced Control Systems',
                    description: 'Master feedback control, stability, and locomotion algorithms',
                    icon: '⚙️'
                  },
                  {
                    title: 'Future Technologies',
                    description: 'Prepare for the next generation of embodied AI systems',
                    icon: '🚀'
                  }
                ].map((feature, index) => (
                  <div
                    key={index}
                    className="p-8 bg-slate-50 dark:bg-slate-800/50 rounded-2xl border border-slate-200 dark:border-slate-700 hover:shadow-xl transition-all duration-300 group"
                  >
                    <div className="text-4xl mb-4 group-hover:scale-110 transition-transform duration-300">
                      {feature.icon}
                    </div>
                    <h3 className="text-xl font-bold text-slate-900 dark:text-white mb-3">
                      {feature.title}
                    </h3>
                    <p className="text-slate-600 dark:text-slate-300">
                      {feature.description}
                    </p>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </section>

        {/* Course Meta Section */}
        <CourseMetaSection />

        {/* Chatbot Teaser Section */}
        <ChatbotTeaser />
      </main>
    </Layout>
  );
}