"""
Database connection utilities for Neon Postgres
"""
import asyncpg
from typing import Optional
import os
from contextlib import asynccontextmanager


class DatabaseConnection:
    def __init__(self, database_url: str):
        self.database_url = database_url
        self.pool = None

    async def connect(self):
        """Create connection pool"""
        self.pool = await asyncpg.create_pool(
            dsn=self.database_url,
            min_size=2,
            max_size=10,
            command_timeout=60
        )

    async def disconnect(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()

    @asynccontextmanager
    async def get_connection(self):
        """Get a connection from the pool"""
        if not self.pool:
            raise RuntimeError("Database not connected")

        conn = await self.pool.acquire()
        try:
            yield conn
        finally:
            await self.pool.release(conn)


# Global database instance
db: Optional[DatabaseConnection] = None


def get_db() -> DatabaseConnection:
    global db
    if not db:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise ValueError("DATABASE_URL environment variable not set")
        db = DatabaseConnection(database_url)
    return db