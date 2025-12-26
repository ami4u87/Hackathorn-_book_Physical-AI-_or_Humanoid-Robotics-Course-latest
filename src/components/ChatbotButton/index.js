import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function ChatbotButton() {
  return (
    <Link to="/chatbot" className={styles.floatingButton} aria-label="Open AI Chatbot">
      <div className={styles.iconWrapper}>
        <svg
          className={styles.icon}
          viewBox="0 0 24 24"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 14.99 3 16.27V22L8.73 19C9.99 19.64 11.46 20 13 20C18.52 20 23 15.52 23 10C23 4.48 18.52 0 13 0C12.66 0 12.33 0.02 12 0.03V2.03C12.33 2.02 12.66 2 13 2C17.42 2 21 5.58 21 10C21 14.42 17.42 18 13 18C11.73 18 10.54 17.66 9.5 17.08L5 19L5 16.08C4.34 14.91 4 13.5 4 12C4 7.58 7.58 4 12 4V2Z"
            fill="currentColor"
          />
          <circle cx="9" cy="12" r="1.5" fill="currentColor"/>
          <circle cx="12" cy="12" r="1.5" fill="currentColor"/>
          <circle cx="15" cy="12" r="1.5" fill="currentColor"/>
        </svg>
      </div>
      <span className={styles.label}>AI Assistant</span>
    </Link>
  );
}
