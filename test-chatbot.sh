#!/bin/bash

# Chatbot Integration Test Script
# Tests the chatbot backend API endpoints

echo "🧪 Testing Physical AI Chatbot Backend"
echo "======================================"
echo ""

# Configuration
BACKEND_URL="${CHATBOT_API_URL:-http://localhost:8000}"

echo "📡 Backend URL: $BACKEND_URL"
echo ""

# Test 1: Health Check
echo "1️⃣  Testing /health endpoint..."
health_response=$(curl -s -w "\nHTTP_STATUS:%{http_code}" "$BACKEND_URL/health")
http_status=$(echo "$health_response" | grep "HTTP_STATUS" | cut -d: -f2)

if [ "$http_status" = "200" ]; then
    echo "✅ Health check passed"
    echo "$health_response" | grep -v "HTTP_STATUS" | jq '.'
else
    echo "❌ Health check failed (Status: $http_status)"
    echo "$health_response" | grep -v "HTTP_STATUS"
fi
echo ""

# Test 2: Stats Endpoint
echo "2️⃣  Testing /stats endpoint..."
stats_response=$(curl -s -w "\nHTTP_STATUS:%{http_code}" "$BACKEND_URL/stats")
http_status=$(echo "$stats_response" | grep "HTTP_STATUS" | cut -d: -f2)

if [ "$http_status" = "200" ]; then
    echo "✅ Stats endpoint passed"
    echo "$stats_response" | grep -v "HTTP_STATUS" | jq '.'

    # Check if Qdrant has data
    chunk_count=$(echo "$stats_response" | grep -v "HTTP_STATUS" | jq -r '.total_chunks')
    if [ "$chunk_count" -gt 0 ]; then
        echo "✅ Qdrant has $chunk_count chunks loaded"
    else
        echo "⚠️  Warning: Qdrant has no chunks. Run 'python scripts/load_content.py'"
    fi
else
    echo "❌ Stats endpoint failed (Status: $http_status)"
    echo "$stats_response" | grep -v "HTTP_STATUS"
fi
echo ""

# Test 3: Query Endpoint
echo "3️⃣  Testing /query endpoint..."
query_response=$(curl -s -w "\nHTTP_STATUS:%{http_code}" \
    -X POST "$BACKEND_URL/query" \
    -H "Content-Type: application/json" \
    -d '{"query": "What is ROS 2?"}')
http_status=$(echo "$query_response" | grep "HTTP_STATUS" | cut -d: -f2)

if [ "$http_status" = "200" ]; then
    echo "✅ Query endpoint passed"
    echo "$query_response" | grep -v "HTTP_STATUS" | jq '.'

    # Show response time
    response_time=$(echo "$query_response" | grep -v "HTTP_STATUS" | jq -r '.response_time_ms')
    echo ""
    echo "⏱️  Response time: ${response_time}ms"
else
    echo "❌ Query endpoint failed (Status: $http_status)"
    echo "$query_response" | grep -v "HTTP_STATUS"
fi
echo ""

# Summary
echo "======================================"
echo "✅ Chatbot Backend Tests Complete"
echo ""
echo "Next steps:"
echo "1. Start frontend: npm start"
echo "2. Visit http://localhost:3000"
echo "3. Click the 💬 button to test the widget"
echo ""
