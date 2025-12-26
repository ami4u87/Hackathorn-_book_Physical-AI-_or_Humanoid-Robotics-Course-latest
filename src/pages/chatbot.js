import React from 'react';
import Layout from '@theme/Layout';
import styles from './chatbot.module.css';

export default function Chatbot() {
  return (
    <Layout
      title="AI Chatbot"
      description="Ask questions about Physical AI and Humanoid Robotics">
      <div className={styles.chatbotContainer}>
        <div className={styles.header}>
          <h1>🤖 AI Course Assistant</h1>
          <p>Ask me anything about ROS 2, robotics, simulation, perception, and humanoid robotics!</p>
        </div>

        <div className={styles.iframeWrapper}>
          <iframe
            src="https://ami4u87-physical-ai-chatbot.hf.space"
            className={styles.chatbotIframe}
            title="Physical AI Chatbot"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowFullScreen
          />
        </div>

        <div className={styles.footer}>
          <p>
            Powered by <a href="https://huggingface.co" target="_blank" rel="noopener noreferrer">Hugging Face</a>
            {' • '}
            Model: <a href="https://huggingface.co/meta-llama/Llama-3.2-3B-Instruct" target="_blank" rel="noopener noreferrer">Llama 3.2</a>
            {' • '}
            Vector DB: Qdrant Cloud
          </p>
        </div>
      </div>
    </Layout>
  );
}
