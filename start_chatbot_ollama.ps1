# Start Chatbot with Ollama (100% Free)
# This script checks if Ollama is ready and starts the chatbot

Write-Host "===============================================" -ForegroundColor Cyan
Write-Host "Physical AI Chatbot - FREE Setup with Ollama" -ForegroundColor Cyan
Write-Host "===============================================`n" -ForegroundColor Cyan

# Check if Ollama is installed
Write-Host "[1/4] Checking Ollama installation..." -ForegroundColor Yellow
$ollamaInstalled = Get-Command ollama -ErrorAction SilentlyContinue

if (-not $ollamaInstalled) {
    Write-Host "  ERROR: Ollama not found!" -ForegroundColor Red
    Write-Host "`n  Please install Ollama first:" -ForegroundColor Yellow
    Write-Host "  1. Visit: https://ollama.com/download/windows"
    Write-Host "  2. Download and run OllamaSetup.exe"
    Write-Host "  3. Run this script again`n"
    exit 1
}

Write-Host "  OK: Ollama is installed" -ForegroundColor Green

# Check if Ollama is running
Write-Host "`n[2/4] Checking if Ollama is running..." -ForegroundColor Yellow
try {
    $response = Invoke-WebRequest -Uri "http://localhost:11434" -UseBasicParsing -TimeoutSec 2
    Write-Host "  OK: Ollama service is running" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Ollama service not responding" -ForegroundColor Red
    Write-Host "`n  Try restarting Ollama:" -ForegroundColor Yellow
    Write-Host "  - Close Ollama from system tray"
    Write-Host "  - Run 'ollama serve' in a new terminal"
    Write-Host "  - Or restart your computer`n"
    exit 1
}

# Check if a model is available
Write-Host "`n[3/4] Checking for available models..." -ForegroundColor Yellow
$models = ollama list

if ($models -match "mistral") {
    Write-Host "  OK: Mistral model found" -ForegroundColor Green
} elseif ($models -match "llama2") {
    Write-Host "  OK: Llama2 model found" -ForegroundColor Green
} else {
    Write-Host "  WARNING: No suitable models found!" -ForegroundColor Yellow
    Write-Host "`n  Please pull a model first:" -ForegroundColor Yellow
    Write-Host "  ollama pull mistral  (Recommended - 4.1 GB)`n"

    $answer = Read-Host "  Would you like to pull Mistral now? (y/n)"
    if ($answer -eq 'y') {
        Write-Host "`n  Pulling Mistral model (this will take a few minutes)..." -ForegroundColor Yellow
        ollama pull mistral
        Write-Host "  OK: Model downloaded successfully" -ForegroundColor Green
    } else {
        Write-Host "  Please pull a model manually and run this script again`n"
        exit 1
    }
}

# Start the chatbot backend
Write-Host "`n[4/4] Starting chatbot backend..." -ForegroundColor Yellow
Write-Host "`n  Backend will start on: http://localhost:8000" -ForegroundColor Cyan
Write-Host "  Frontend is already running on: http://localhost:3000`n" -ForegroundColor Cyan

Write-Host "  Press Ctrl+C to stop the backend`n" -ForegroundColor Gray

# Kill existing uvicorn processes
Get-Process -Name uvicorn -ErrorAction SilentlyContinue | Stop-Process -Force

# Start the backend
Set-Location "backend\chatbot"
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
