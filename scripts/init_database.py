#!/usr/bin/env python3
"""
Neon Postgres schema initialization script
Creates all required tables for the Physical AI chatbot application

Usage:
    python scripts/init_database.py
    python scripts/init_database.py --database-url "postgresql://..."

Environment Variables:
    DATABASE_URL: PostgreSQL connection string (required if not passed as argument)
"""
import psycopg2
import os
import sys
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


def init_database(database_url: Optional[str] = None) -> None:
    """
    Initialize the database schema with all required tables
    """
    # Use environment variable or provided parameter
    db_url = database_url or os.getenv("DATABASE_URL")

    if not db_url:
        print("[ERROR] Error: DATABASE_URL not found")
        print("   Please set DATABASE_URL in your .env file or pass --database-url argument")
        sys.exit(1)

    print(f"Connecting to database...")

    # Connect to the database
    try:
        conn = psycopg2.connect(db_url)
        print("[OK] Successfully connected to PostgreSQL database")
    except psycopg2.OperationalError as e:
        print(f"[ERROR] Failed to connect to database: {e}")
        print("\nTroubleshooting:")
        print("1. Check DATABASE_URL in .env file")
        print("2. Verify database is running (for local Postgres)")
        print("3. Ensure SSL mode is correct (Neon requires sslmode=require)")
        sys.exit(1)

    cursor = conn.cursor()

    try:
        # Create students table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS students (
                student_id VARCHAR(255) PRIMARY KEY,
                jwt_secret VARCHAR(512),
                quota_daily INT DEFAULT 100,
                quota_used_today INT DEFAULT 0,
                last_reset DATE DEFAULT CURRENT_DATE,
                created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                last_active TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                total_queries INTEGER DEFAULT 0,
                is_active BOOLEAN DEFAULT TRUE
            )
        """)
        conn.commit()
        print("[OK] Created students table")

        # Create chatbot_queries table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chatbot_queries (
                query_id SERIAL PRIMARY KEY,
                student_id VARCHAR(255) REFERENCES students(student_id) ON DELETE CASCADE,
                query_text TEXT NOT NULL,
                retrieved_chunks INTEGER DEFAULT 0,
                response_text TEXT NOT NULL,
                citations JSONB DEFAULT '[]'::jsonb,
                feedback_rating INT CHECK (feedback_rating BETWEEN 1 AND 5),
                feedback_text TEXT,
                response_time_ms INT,
                model_used VARCHAR(100) DEFAULT 'gpt-4-turbo-preview',
                tokens_used INTEGER,
                context_module VARCHAR(255),
                created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                flagged BOOLEAN DEFAULT FALSE
            )
        """)
        conn.commit()
        print("[OK] Created chatbot_queries table")

        # Create indexes for chatbot_queries
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_queries_student ON chatbot_queries(student_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_queries_created_at ON chatbot_queries(created_at DESC)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_queries_context_module ON chatbot_queries(context_module)
        """)
        conn.commit()
        print("[OK] Created indexes for chatbot_queries")

        # Create api_usage_logs table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS api_usage_logs (
                log_id SERIAL PRIMARY KEY,
                student_id VARCHAR(255) REFERENCES students(student_id) ON DELETE CASCADE,
                endpoint VARCHAR(255),
                method VARCHAR(10),
                service VARCHAR(50) DEFAULT 'chatbot',
                request_timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                response_timestamp TIMESTAMP WITH TIME ZONE,
                request_tokens INT DEFAULT 0,
                response_tokens INT DEFAULT 0,
                cost_usd DECIMAL(10,6) DEFAULT 0.0,
                quota_remaining INT,
                status_code INT,
                ip_address INET,
                user_agent TEXT
            )
        """)
        conn.commit()
        print("[OK] Created api_usage_logs table")

        # Create indexes for api_usage_logs
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_usage_student_timestamp
            ON api_usage_logs(student_id, request_timestamp DESC)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_usage_created_at
            ON api_usage_logs(request_timestamp DESC)
        """)
        conn.commit()
        print("[OK] Created indexes for api_usage_logs")

        # Create exercise_submissions table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS exercise_submissions (
                submission_id SERIAL PRIMARY KEY,
                student_id VARCHAR(255) REFERENCES students(student_id) ON DELETE CASCADE,
                exercise_id VARCHAR(255) NOT NULL,
                module_id VARCHAR(255),
                validation_hash VARCHAR(64) NOT NULL,
                submission_text TEXT,
                code_files JSONB DEFAULT '{}'::jsonb,
                submitted_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                is_correct BOOLEAN,
                attempt_number INT DEFAULT 1,
                validation_status VARCHAR(50) DEFAULT 'pending',
                validation_feedback TEXT,
                UNIQUE(student_id, exercise_id)
            )
        """)
        conn.commit()
        print("[OK] Created exercise_submissions table")

        # Create indexes for exercise_submissions
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_submissions_student_exercise
            ON exercise_submissions(student_id, exercise_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_submissions_module
            ON exercise_submissions(module_id)
        """)
        conn.commit()
        print("[OK] Created indexes for exercise_submissions")

        # Verify tables were created
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_type = 'BASE TABLE'
            ORDER BY table_name
        """)
        tables = [row[0] for row in cursor.fetchall()]

        print("\n" + "="*60)
        print("Database schema initialized successfully!")
        print("="*60)
        print("\nTables created:")
        print("  [OK] students - Student information and quotas")
        print("  [OK] chatbot_queries - Chatbot queries and responses")
        print("  [OK] api_usage_logs - API usage tracking")
        print("  [OK] exercise_submissions - Exercise validation")
        print(f"\nTotal tables in database: {len(tables)}")
        print("="*60)
        print("\nNext steps:")
        print("1. Initialize Qdrant collection: python scripts/init_qdrant.py")
        print("2. Load course content into vector database")
        print("3. Start backend services: cd backend && docker-compose up -d")
        print()

    except psycopg2.Error as e:
        print(f"[ERROR] Error initializing database: {e}")
        conn.rollback()
        raise
    finally:
        cursor.close()
        conn.close()


def main() -> None:
    """Main entry point for CLI usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Initialize PostgreSQL schema for Physical AI chatbot"
    )
    parser.add_argument(
        "--database-url",
        default=None,
        help="Database connection string (default: from DATABASE_URL env var)"
    )

    args = parser.parse_args()

    print("\n" + "="*60)
    print("Physical AI Chatbot - Database Schema Initialization")
    print("="*60 + "\n")

    # Run the initialization
    try:
        init_database(database_url=args.database_url)
    except Exception as e:
        print(f"\n[ERROR] Failed to initialize database: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()