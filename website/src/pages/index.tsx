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
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener('scroll', handleScroll, { passive: true });

    const handleMouseMove = (e: MouseEvent) => {
      setMousePosition({
        x: (e.clientX / window.innerWidth) * 100,
        y: (e.clientY / window.innerHeight) * 100,
      });
    };

    window.addEventListener('mousemove', handleMouseMove);

    return () => {
      window.removeEventListener('scroll', handleScroll);
      window.removeEventListener('mousemove', handleMouseMove);
    };
  }, []);

  return (
    <header
      className={clsx(
        "relative overflow-hidden bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white min-h-screen flex items-center",
        "transition-all duration-300"
      )}
    >
      {/* Enhanced animated background with parallax effect */}
      <div className="absolute inset-0 overflow-hidden">
        {/* Moving particles */}
        <div
          className="absolute w-64 h-64 bg-purple-500/10 rounded-full blur-2xl animate-pulse"
          style={{
            top: `${25 + mousePosition.y * 0.1}%`,
            left: `${25 + mousePosition.x * 0.1}%`,
            transition: 'all 0.3s ease-out',
          }}
        ></div>
        <div
          className="absolute w-72 h-72 bg-blue-500/10 rounded-full blur-2xl animate-pulse delay-1000"
          style={{
            bottom: `${25 - mousePosition.y * 0.05}%`,
            right: `${25 - mousePosition.x * 0.05}%`,
            transition: 'all 0.3s ease-out',
          }}
        ></div>
        <div
          className="absolute w-80 h-80 bg-indigo-500/10 rounded-full blur-2xl animate-pulse delay-500"
          style={{
            top: `${50 + mousePosition.y * 0.05}%`,
            left: `${50 + mousePosition.x * 0.05}%`,
            transition: 'all 0.3s ease-out',
          }}
        ></div>

        {/* Grid pattern with subtle animation */}
        <div
          className="absolute inset-0 opacity-10"
          style={{
            backgroundImage: `radial-gradient(circle at 1px 1px, white 1px, transparent 0)`,
            backgroundSize: '50px 50px',
            transform: `translate(${scrollY * 0.05}px, ${scrollY * 0.05}px)`,
          }}
        ></div>

        {/* Floating geometric shapes */}
        <div className="absolute inset-0 overflow-hidden pointer-events-none">
          <div className="absolute top-20 left-10 w-2 h-2 bg-white/30 rounded-full animate-ping"></div>
          <div className="absolute top-40 right-20 w-1 h-1 bg-blue-300/50 rounded-full animate-pulse"></div>
          <div className="absolute bottom-32 left-32 w-1.5 h-1.5 bg-purple-300/50 rounded-full animate-bounce"></div>
          <div className="absolute top-60 left-1/4 w-0.5 h-20 bg-white/10 rotate-45 animate-pulse"></div>
        </div>
      </div>

      <div className="relative container mx-auto px-4 py-16">
        <div className="max-w-6xl mx-auto text-center">
          {/* Hero icon with enhanced animation */}
          <div className="mb-8">
            <div className="inline-flex items-center justify-center w-24 h-24 bg-gradient-to-r from-blue-500 via-purple-500 to-cyan-500 rounded-2xl mb-8 shadow-2xl border border-white/20 relative overflow-hidden">
              <div className="absolute inset-0 bg-gradient-to-r from-transparent via-white/10 to-transparent animate-pulse"></div>
              <svg className="w-12 h-12 text-white z-10" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
            </div>
          </div>

          {/* Enhanced title with better gradient and typography */}
          <h1 className="text-5xl md:text-7xl lg:text-8xl xl:text-9xl font-extrabold mb-8 hero__title bg-clip-text text-transparent bg-gradient-to-r from-blue-300 via-purple-300 to-cyan-300 leading-tight tracking-tight">
            {siteConfig.title}
          </h1>

          {/* Enhanced subtitle with better readability */}
          <p className="text-xl md:text-3xl lg:text-4xl mb-16 text-blue-100/90 hero__subtitle max-w-5xl mx-auto leading-relaxed font-light">
            {siteConfig.tagline}
          </p>

          {/* Enhanced CTA buttons with better hover effects */}
          <div className="flex flex-col sm:flex-row gap-6 justify-center items-center mb-16">
            <Link
              className={clsx(
                "group relative px-10 py-5 bg-gradient-to-r from-blue-600 to-purple-600 text-white font-bold text-xl rounded-2xl shadow-2xl",
                "hover:shadow-blue-500/25 transform hover:-translate-y-1 transition-all duration-300",
                "w-full sm:w-auto text-center overflow-hidden",
                "before:absolute before:inset-0 before:bg-white/20 before:opacity-0 before:transition-opacity before:duration-300",
                "before:hover:opacity-100",
                "after:absolute after:inset-0 after:rounded-2xl after:border after:border-white/10"
              )}
              to="/docs/module-1/">
              <span className="relative z-10 inline-flex items-center justify-center gap-4">
                <svg className="w-6 h-6 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
                <span>Begin Journey</span>
              </span>
            </Link>

            <Link
              className={clsx(
                "group relative px-10 py-5 bg-white/10 backdrop-blur-md text-white font-semibold text-xl rounded-2xl border border-white/20",
                "hover:bg-white/20 transition-all duration-300",
                "w-full sm:w-auto text-center overflow-hidden",
                "before:absolute before:inset-0 before:bg-white/10 before:opacity-0 before:transition-opacity before:duration-300",
                "before:hover:opacity-100"
              )}
              to="/docs/intro">
              <span className="relative z-10 inline-flex items-center justify-center gap-4">
                <svg className="w-6 h-6 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                </svg>
                <span>Explore Syllabus</span>
              </span>
            </Link>
          </div>

          {/* Enhanced course summary with better spacing and typography */}
          <div className="courseSummary max-w-4xl mx-auto">
            <p className="text-lg md:text-xl text-blue-100/80 leading-relaxed font-light">
              This comprehensive textbook explores the intersection of Artificial Intelligence and Robotics,
              focusing on how intelligent systems can interact with and operate in the physical world.
              From fundamental concepts of Physical AI to advanced humanoid robotics, you'll gain deep
              insights into the future of embodied intelligence.
            </p>
          </div>

          {/* Scroll indicator */}
          <div className="mt-20 flex justify-center">
            <div className="flex flex-col items-center text-blue-200/70">
              <span className="text-sm mb-2">Scroll to explore</span>
              <svg className="w-6 h-10 animate-bounce" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 14l-7 7m0 0l-7-7m7 7V3" />
              </svg>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// Updated module data with more compelling descriptions
const moduleData: ModuleData[] = [
  {
    id: 'module-1',
    title: 'Module 1: Foundations of Physical AI',
    description: 'Discover the core principles of Physical AI and understand how intelligence manifests differently when operating in real-world physical systems.',
    path: '/docs/module-1/'
  },
  {
    id: 'module-2',
    title: 'Module 2: Kinematics & Dynamics',
    description: 'Master the mathematical foundations of robot movement and dive deep into the physics governing robotic systems and their interactions.',
    path: '/docs/module-2/'
  },
  {
    id: 'module-3',
    title: 'Module 3: Control Systems & Locomotion',
    description: 'Explore sophisticated feedback control mechanisms, stability theory, and how robots achieve precise, coordinated movement patterns.',
    path: '/docs/module-3/'
  },
  {
    id: 'module-4',
    title: 'Module 4: Perception & Machine Learning',
    description: 'Unlock how robots interpret their environment and leverage machine learning to adapt and improve their performance over time.',
    path: '/docs/module-4/'
  }
];

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    // Intersection Observer for better animation triggers
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            setIsVisible(true);
          }
        });
      },
      { threshold: 0.1 }
    );

    const element = document.querySelector('main');
    if (element) {
      observer.observe(element);
    }

    return () => {
      if (element) {
        observer.unobserve(element);
      }
    };
  }, []);

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Educational textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />

      {/* Module Cards Section */}
      <section className="py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="max-w-4xl mx-auto text-center mb-16">
            <div className="inline-flex items-center justify-center w-20 h-20 bg-gradient-to-r from-blue-600 to-purple-600 rounded-2xl mb-6 shadow-2xl mx-auto">
              <svg className="w-10 h-10 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-3 7h3m-3 4h3m-6-4h.01M9 16h.01" />
              </svg>
            </div>
            <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-slate-900 dark:text-white mb-4 sectionTitle">
              Your Learning Journey
            </h2>
            <p className="text-lg md:text-xl text-slate-600 dark:text-slate-300 sectionSubtitle max-w-2xl mx-auto leading-relaxed mb-12">
              Progress through our structured curriculum from foundational concepts to advanced humanoid robotics
            </p>

            {/* Learning Journey Overview */}
            <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-5xl mx-auto mb-16">
              <div className="bg-white dark:bg-slate-800/50 backdrop-blur-sm rounded-2xl p-6 border border-slate-200 dark:border-slate-700/50">
                <div className="w-12 h-12 bg-blue-100 dark:bg-blue-900/30 rounded-xl flex items-center justify-center mb-4 mx-auto">
                  <span className="text-blue-600 dark:text-blue-400 text-xl font-bold">1</span>
                </div>
                <h3 className="text-lg font-bold text-slate-900 dark:text-white mb-2">Foundation Building</h3>
                <p className="text-slate-600 dark:text-slate-300 text-sm">
                  Start with core concepts of Physical AI and robotics fundamentals
                </p>
              </div>

              <div className="bg-white dark:bg-slate-800/50 backdrop-blur-sm rounded-2xl p-6 border border-slate-200 dark:border-slate-700/50">
                <div className="w-12 h-12 bg-purple-100 dark:bg-purple-900/30 rounded-xl flex items-center justify-center mb-4 mx-auto">
                  <span className="text-purple-600 dark:text-purple-400 text-xl font-bold">2</span>
                </div>
                <h3 className="text-lg font-bold text-slate-900 dark:text-white mb-2">Skill Development</h3>
                <p className="text-slate-600 dark:text-slate-300 text-sm">
                  Deepen your knowledge with advanced kinematics and control systems
                </p>
              </div>

              <div className="bg-white dark:bg-slate-800/50 backdrop-blur-sm rounded-2xl p-6 border border-slate-200 dark:border-slate-700/50">
                <div className="w-12 h-12 bg-green-100 dark:bg-green-900/30 rounded-xl flex items-center justify-center mb-4 mx-auto">
                  <span className="text-green-600 dark:text-green-400 text-xl font-bold">3</span>
                </div>
                <h3 className="text-lg font-bold text-slate-900 dark:text-white mb-2">Mastery Phase</h3>
                <p className="text-slate-600 dark:text-slate-300 text-sm">
                  Apply everything in advanced AI for humanoid systems and capstone projects
                </p>
              </div>
            </div>
          </div>

          {/* Module Cards */}
          <div className="max-w-4xl mx-auto text-center mb-12">
            <h3 className="text-2xl md:text-3xl font-bold text-slate-900 dark:text-white mb-6">
              Explore Our Curriculum
            </h3>
            <p className="text-slate-600 dark:text-slate-300 max-w-xl mx-auto">
              Each module builds upon the previous one, ensuring a comprehensive understanding of Physical AI & Humanoid Robotics
            </p>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-8 max-w-6xl mx-auto">
            {moduleData.map((module, index) => (
              <div
                key={module.id}
                className={clsx(
                  "transform transition-all duration-700 ease-out group",
                  isVisible ? "translate-y-0 opacity-100" : "translate-y-10 opacity-0",
                  `delay-${index * 150}`
                )}
                style={{ transitionDelay: `${index * 150}ms` }}
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

          {/* Progression Arrow */}
          <div className="max-w-6xl mx-auto mt-16 flex justify-center">
            <div className="flex flex-col items-center">
              <div className="flex items-center gap-4 text-slate-400 dark:text-slate-500 mb-4">
                <span className="text-sm font-medium">Learning Path</span>
                <svg className="w-8 h-8 animate-bounce" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 14l-7 7m0 0l-7-7m7 7V3" />
                </svg>
                <span className="text-sm font-medium">Start → Finish</span>
              </div>
              <div className="w-24 h-1 bg-gradient-to-r from-blue-500 to-purple-500 rounded-full"></div>
            </div>
          </div>
        </div>
      </section>

      {/* Enhanced Stats Section */}
      <section className="py-24 bg-gradient-to-r from-slate-50 via-purple-50 to-slate-50 dark:from-slate-800 dark:to-slate-900">
        <div className="container mx-auto px-4">
          <div className="max-w-6xl mx-auto">
            <div className="grid grid-cols-2 md:grid-cols-4 gap-8 text-center">
              {[
                { value: '4', label: 'Core Modules', icon: '📚' },
                { value: '12+', label: 'Weeks Content', icon: '📅' },
                { value: '50+', label: 'Detailed Lessons', icon: '✍️' },
                { value: '∞', label: 'Learning Possibilities', icon: '🚀' }
              ].map((stat, index) => (
                <div
                  key={index}
                  className="group p-10 bg-white/70 dark:bg-slate-800/60 backdrop-blur-sm rounded-3xl border border-white/30 dark:border-slate-700/60 hover:bg-white/90 dark:hover:bg-slate-800/80 transition-all duration-500 hover:scale-105"
                >
                  <div className="text-5xl md:text-6xl font-bold text-blue-600 dark:text-blue-400 mb-6 group-hover:scale-110 transition-transform duration-300">
                    {stat.value}
                  </div>
                  <div className="text-4xl mb-4">{stat.icon}</div>
                  <div className="text-slate-700 dark:text-slate-300 font-semibold text-lg">
                    {stat.label}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Enhanced Features Section */}
      <section className="py-24 bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white">
        <div className="container mx-auto px-4">
          <div className="max-w-6xl mx-auto">
            <div className="text-center mb-20">
              <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold mb-8">
                Transform Your Understanding of AI & Robotics
              </h2>
              <p className="text-xl text-blue-200 max-w-3xl mx-auto">
                Master the cutting-edge intersection of artificial intelligence and physical systems with hands-on learning
              </p>
            </div>

            <div className="grid grid-cols-1 lg:grid-cols-3 gap-10">
              {[
                {
                  title: 'Real-World Applications',
                  description: 'Apply theoretical concepts to practical robotics challenges and understand how AI functions in physical environments',
                  icon: '🌍'
                },
                {
                  title: 'Advanced Control Systems',
                  description: 'Develop expertise in feedback control, system stability, and locomotion algorithms for complex robotic systems',
                  icon: '⚙️'
                },
                {
                  title: 'Future Technologies',
                  description: 'Prepare for tomorrow\'s breakthroughs in embodied AI, humanoid robots, and human-robot interaction',
                  icon: '🤖'
                }
              ].map((feature, index) => (
                <div
                  key={index}
                  className="p-10 bg-white/10 backdrop-blur-md rounded-3xl border border-white/20 hover:bg-white/15 transition-all duration-500 group hover:scale-[1.02]"
                >
                  <div className="text-5xl mb-6 group-hover:scale-110 transition-transform duration-300 text-blue-300">
                    {feature.icon}
                  </div>
                  <h3 className="text-2xl font-bold mb-4">
                    {feature.title}
                  </h3>
                  <p className="text-blue-100 text-lg leading-relaxed">
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
    </Layout>
  );
}