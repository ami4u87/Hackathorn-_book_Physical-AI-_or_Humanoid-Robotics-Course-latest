# Chatbot Integration Test Script (PowerShell)
# Tests the chatbot backend API endpoints

Write-Host "🧪 Testing Physical AI Chatbot Backend" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Configuration
$BACKEND_URL = if ($env:CHATBOT_API_URL) { $env:CHATBOT_API_URL } else { "http://localhost:8000" }

Write-Host "📡 Backend URL: $BACKEND_URL"
Write-Host ""

# Test 1: Health Check
Write-Host "1️⃣  Testing /health endpoint..." -ForegroundColor Yellow
try {
    $health = Invoke-RestMethod -Uri "$BACKEND_URL/health" -Method Get
    Write-Host "✅ Health check passed" -ForegroundColor Green
    $health | ConvertTo-Json -Depth 10
} catch {
    Write-Host "❌ Health check failed" -ForegroundColor Red
    Write-Host $_.Exception.Message
}
Write-Host ""

# Test 2: Stats Endpoint
Write-Host "2️⃣  Testing /stats endpoint..." -ForegroundColor Yellow
try {
    $stats = Invoke-RestMethod -Uri "$BACKEND_URL/stats" -Method Get
    Write-Host "✅ Stats endpoint passed" -ForegroundColor Green
    $stats | ConvertTo-Json -Depth 10

    # Check if Qdrant has data
    if ($stats.total_chunks -gt 0) {
        Write-Host "✅ Qdrant has $($stats.total_chunks) chunks loaded" -ForegroundColor Green
    } else {
        Write-Host "⚠️  Warning: Qdrant has no chunks. Run 'python scripts/load_content.py'" -ForegroundColor Yellow
    }
} catch {
    Write-Host "❌ Stats endpoint failed" -ForegroundColor Red
    Write-Host $_.Exception.Message
}
Write-Host ""

# Test 3: Query Endpoint
Write-Host "3️⃣  Testing /query endpoint..." -ForegroundColor Yellow
try {
    $body = @{
        query = "What is ROS 2?"
    } | ConvertTo-Json

    $query = Invoke-RestMethod -Uri "$BACKEND_URL/query" -Method Post -Body $body -ContentType "application/json"
    Write-Host "✅ Query endpoint passed" -ForegroundColor Green
    $query | ConvertTo-Json -Depth 10

    Write-Host ""
    Write-Host "⏱️  Response time: $($query.response_time_ms)ms" -ForegroundColor Cyan
} catch {
    Write-Host "❌ Query endpoint failed" -ForegroundColor Red
    Write-Host $_.Exception.Message
}
Write-Host ""

# Summary
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "✅ Chatbot Backend Tests Complete" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:"
Write-Host "1. Start frontend: npm start"
Write-Host "2. Visit http://localhost:3000"
Write-Host "3. Click the 💬 button to test the widget"
Write-Host ""
