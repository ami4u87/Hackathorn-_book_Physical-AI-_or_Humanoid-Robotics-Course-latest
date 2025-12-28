# Install Ollama on Windows
Write-Host "===============================================" -ForegroundColor Cyan
Write-Host "Installing Ollama - 100% Free Local LLM" -ForegroundColor Cyan
Write-Host "===============================================`n" -ForegroundColor Cyan

# Download Ollama installer
Write-Host "[1/3] Downloading Ollama installer..." -ForegroundColor Yellow
$installerPath = "$env:TEMP\OllamaSetup.exe"

try {
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "https://ollama.com/download/OllamaSetup.exe" -OutFile $installerPath -UseBasicParsing

    $fileSize = [math]::Round((Get-Item $installerPath).Length / 1MB, 2)
    Write-Host "  OK: Downloaded ($fileSize MB)" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Failed to download installer" -ForegroundColor Red
    Write-Host "  Please download manually from: https://ollama.com/download/windows" -ForegroundColor Yellow
    exit 1
}

# Run installer
Write-Host "`n[2/3] Running installer..." -ForegroundColor Yellow
Write-Host "  NOTE: You may need to click through the installer windows" -ForegroundColor Gray

try {
    Start-Process -FilePath $installerPath -Wait
    Write-Host "  OK: Installation completed" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Installation failed" -ForegroundColor Red
    exit 1
}

# Wait for Ollama service to start
Write-Host "`n[3/3] Waiting for Ollama service to start..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

# Verify installation
$maxRetries = 10
$retryCount = 0
$ollamaRunning = $false

while ($retryCount -lt $maxRetries -and -not $ollamaRunning) {
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:11434" -UseBasicParsing -TimeoutSec 2
        if ($response.StatusCode -eq 200) {
            $ollamaRunning = $true
            Write-Host "  OK: Ollama is running!" -ForegroundColor Green
        }
    } catch {
        $retryCount++
        Write-Host "  Waiting... ($retryCount/$maxRetries)" -ForegroundColor Gray
        Start-Sleep -Seconds 2
    }
}

if (-not $ollamaRunning) {
    Write-Host "  WARNING: Ollama service may not have started automatically" -ForegroundColor Yellow
    Write-Host "  Try running: ollama serve" -ForegroundColor Yellow
}

Write-Host "`n===============================================" -ForegroundColor Cyan
Write-Host "Ollama Installation Complete!" -ForegroundColor Green
Write-Host "===============================================`n" -ForegroundColor Cyan

Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Pull a model: ollama pull mistral"
Write-Host "2. Start chatbot: .\start_chatbot_ollama.ps1`n"
