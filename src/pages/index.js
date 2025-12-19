import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={styles.heroBackground}></div>
      <div className={clsx('container', styles.heroContainer)}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>Build the Future of Intelligent Machines</h1>
          <p className={styles.heroSubtitle}>
            Master Physical AI and Humanoid Robotics with hands-on projects,
            simulation tools, and real-world robotics workflows — from fundamentals to advanced systems.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning
            </Link>
            <Link
              className={clsx('button button--outline button--secondary button--lg', styles.buttonSecondary)}
              to="/docs/intro">
              View Curriculum
            </Link>
          </div>
          <div className={styles.trustElements}>
            <span className={styles.trustBadge}>🤖 Hands-on ROS 2 Projects</span>
            <span className={styles.trustBadge}>🎯 Industry-Standard Tools</span>
            <span className={styles.trustBadge}>🚀 From Basics to Advanced</span>
          </div>
        </div>
        <div className={styles.heroImageContainer}>
          <div className={styles.heroImagePlaceholder}>
            <img
              src="/img/robot-hero.png"
              alt="Advanced Humanoid Robot"
              className={styles.heroImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      icon: '⚙️',
      title: 'ROS 2 Fundamentals',
      description: (
        <>
          Master the core concepts of ROS 2 including publishers, subscribers,
          services, actions, parameters, and coordinate transformations.
        </>
      ),
    },
    {
      icon: '🎮',
      title: 'Simulation Environments',
      description: (
        <>
          Learn to use Gazebo, Unity, and NVIDIA Isaac Sim for realistic
          robot simulation and testing before deployment.
        </>
      ),
    },
    {
      icon: '🧠',
      title: 'Vision-Language-Action Models',
      description: (
        <>
          Integrate cutting-edge VLA models with robotic systems for
          intelligent, context-aware physical AI applications.
        </>
      ),
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <h2 className={styles.featuresTitle}>What You'll Learn</h2>
          <p className={styles.featuresSubtitle}>
            Build production-ready robotics skills through hands-on projects
          </p>
        </div>
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--4')}>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>{feature.icon}</div>
                <h3 className={styles.featureTitle}>{feature.title}</h3>
                <p className={styles.featureDescription}>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A comprehensive course on embodied intelligence and humanoid robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
