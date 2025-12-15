#!/usr/bin/env python3
"""
Neon Postgres schema initialization script
Creates all required tables for the application
"""
import asyncpg
import os
import sys
from typing import Optional


async def init_database(database_url: Optional[str] = None):
    """
    Initialize the database schema with all required tables
    """
    # Use environment variable or provided parameter
    db_url = database_url or os.getenv("DATABASE_URL")

    if not db_url:
        raise ValueError("DATABASE_URL environment variable not set")

    print(f"Connecting to database: {db_url}")

    # Connect to the database
    conn = await asyncpg.connect(db_url)

    try:
        # Create students table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS students (
                student_id VARCHAR(50) PRIMARY KEY,
                jwt_secret VARCHAR(255),
                quota_daily INT DEFAULT 100,
                quota_used_today INT DEFAULT 0,
                last_reset DATE DEFAULT CURRENT_DATE,
                created_at TIMESTAMP DEFAULT NOW(),
                last_active TIMESTAMP
            )
        """)
        print("✓ Created students table")

        # Create chatbot_queries table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chatbot_queries (
                query_id UUID PRIMARY KEY,
                student_id VARCHAR(50) REFERENCES students(student_id),
                query_text TEXT NOT NULL,
                retrieved_chunks JSONB NOT NULL,
                response_text TEXT NOT NULL,
                citations JSONB NOT NULL,
                feedback_rating INT CHECK (feedback_rating BETWEEN 1 AND 5),
                feedback_text TEXT,
                response_time_ms INT,
                timestamp TIMESTAMP DEFAULT NOW()
            )
        """)
        print("✓ Created chatbot_queries table")

        # Create indexes for chatbot_queries
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_queries_student ON chatbot_queries(student_id);
        """)
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_queries_timestamp ON chatbot_queries(timestamp);
        """)
        print("✓ Created indexes for chatbot_queries")

        # Create api_usage_logs table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS api_usage_logs (
                log_id SERIAL PRIMARY KEY,
                student_id VARCHAR(50) REFERENCES students(student_id),
                endpoint VARCHAR(100),
                service VARCHAR(20) CHECK (service IN ('chatbot', 'vla')),
                request_timestamp TIMESTAMP DEFAULT NOW(),
                response_timestamp TIMESTAMP,
                request_tokens INT,
                response_tokens INT,
                cost_usd DECIMAL(10,6),
                quota_remaining INT,
                status_code INT
            )
        """)
        print("✓ Created api_usage_logs table")

        # Create indexes for api_usage_logs
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_usage_student_date ON api_usage_logs(student_id, DATE(request_timestamp));
        """)
        print("✓ Created indexes for api_usage_logs")

        # Create exercise_submissions table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS exercise_submissions (
                submission_id UUID PRIMARY KEY,
                student_id VARCHAR(50) REFERENCES students(student_id),
                exercise_id VARCHAR(100) NOT NULL,
                validation_hash VARCHAR(64) NOT NULL,
                timestamp TIMESTAMP DEFAULT NOW(),
                is_correct BOOLEAN,
                attempt_number INT
            )
        """)
        print("✓ Created exercise_submissions table")

        # Create indexes for exercise_submissions
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_submissions_student_exercise ON exercise_submissions(student_id, exercise_id);
        """)
        print("✓ Created indexes for exercise_submissions")

        print("\nDatabase schema initialized successfully!")
        print("\nTables created:")
        print("- students: Stores student information and quotas")
        print("- chatbot_queries: Logs chatbot queries and responses")
        print("- api_usage_logs: Tracks API usage for rate limiting")
        print("- exercise_submissions: Records exercise validation attempts")

    except Exception as e:
        print(f"Error initializing database: {e}")
        raise
    finally:
        await conn.close()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Initialize Neon Postgres schema")
    parser.add_argument("--database-url", default=None,
                       help="Database URL (default: from DATABASE_URL env var)")

    args = parser.parse_args()

    # Run the async function
    try:
        import asyncio
        asyncio.run(init_database(database_url=args.database_url))
    except Exception as e:
        print(f"Failed to initialize database: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()