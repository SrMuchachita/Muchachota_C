# publish.ps1 — Compila, sube Release a GitHub y actualiza version.json
# Uso: .\publish.ps1 "Descripcion de las mejoras de esta version"

param(
    [string]$Notes = ""
)

$match = Select-String -Path "CMakeLists.txt" -Pattern 'project\(\w+ VERSION ([\d.]+)\)'
if (-not $match) { Write-Error "No se encontro VERSION en CMakeLists.txt"; exit 1 }
$version = $match.Matches[0].Groups[1].Value
Write-Host "`n>>> Version detectada: $version`n" -ForegroundColor Cyan

Write-Host ">>> Compilando..." -ForegroundColor Yellow
idf.py build
if ($LASTEXITCODE -ne 0) { Write-Error "Build fallo"; exit 1 }

if (-not $Notes) { $Notes = "Version $version" }

Write-Host "`n>>> Creando Release v$version en GitHub..." -ForegroundColor Yellow
gh release create "v$version" "build/WP_C_V9_UP.bin" `
    --title "v$version" `
    --notes "$Notes"
if ($LASTEXITCODE -ne 0) { Write-Error "No se pudo crear el Release"; exit 1 }

Write-Host "`n>>> Actualizando version.json..." -ForegroundColor Yellow
Set-Content -Path "version.json" -Value "{`"version`":`"$version`"}" -NoNewline

git add version.json
git commit -m "v$version"
git push

Write-Host "`n>>> Listo! El ESP32 se actualizara en el proximo chequeo.`n" -ForegroundColor Green
