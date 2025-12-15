import React, { useEffect, useRef, useState } from 'react';

export default function ChatbotEmbed({ apiUrl }) {
  const [isVisible, setIsVisible] = useState(false);
  const containerRef = useRef(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.disconnect();
        }
      },
      { threshold: 0.1 }
    );

    if (containerRef.current) {
      observer.observe(containerRef.current);
    }

    return () => observer.disconnect();
  }, []);

  return (
    <div ref={containerRef} style={{ minHeight: '400px' }}>
      {isVisible && (
        <iframe
          src={apiUrl}
          title="Course Chatbot"
          style={{ width: '100%', height: '400px', border: 'none' }}
          aria-label="Interactive course chatbot for questions"
        />
      )}
    </div>
  );
}