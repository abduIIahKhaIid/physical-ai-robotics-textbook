import React from 'react';
import Layout from '@theme/Layout';
import { useColorMode } from '@docusaurus/theme-common';
import HeroSection from '../components/HeroSection';
import StatsBar from '../components/StatsBar';
import CurriculumOverview from '../components/CurriculumOverview';
import ValueProposition from '../components/ValueProposition';
import CourseMetaSection from '../components/CourseMetaSection';
import CTABanner from '../components/CTABanner';

function HomepageContent() {
  const { colorMode } = useColorMode();
  const isDarkTheme = colorMode === 'dark';

  return (
    <main>
      <HeroSection isDarkTheme={isDarkTheme} />
      <StatsBar />
      <CurriculumOverview />
      <ValueProposition />
      <CourseMetaSection />
      <CTABanner />
    </main>
  );
}

export default function RoboticsLandingPage() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="An interactive, open-source textbook bridging theory and practice in Physical AI and Humanoid Robotics"
    >
      <HomepageContent />
    </Layout>
  );
}
