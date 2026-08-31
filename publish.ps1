param(
    [string]$Notes = ""
)

$ErrorActionPreference = "Stop"

$ProjectName = "WP_C_V9_UP"

# --- Leer version desde project(NOMBRE VERSION x.x.x) en CMakeLists.txt ---
$cmakeContent = Get-Content -Raw "CMakeLists.txt"
if ($cmakeContent -notmatch 'project\s*\(\s*\S+\s+VERSION\s+([0-9]+\.[0-9]+\.[0-9]+)') {
    throw "No se encontro 'project(... VERSION x.x.x)' en CMakeLists.txt"
}
$version = $Matches[1]

if ([string]::IsNullOrWhiteSpace($Notes)) {
    $Notes = "Version $version"
}

Write-Host "Publicando version $version ..." -ForegroundColor Cyan

# --- Compilar ---
idf.py build
if ($LASTEXITCODE -ne 0) { throw "idf.py build fallo" }

$binPath = "build/$ProjectName.bin"
if (-not (Test-Path $binPath)) {
    throw "No se encontro $binPath luego del build"
}

# --- Crear Release en GitHub y subir el .bin con su nombre real ---
gh release create "v$version" "$binPath" --title "v$version" --notes "$Notes"
if ($LASTEXITCODE -ne 0) { throw "gh release create fallo" }

# --- Actualizar version.json ---
Set-Content -Path "version.json" -Value "{`"version`":`"$version`"}" -Encoding utf8 -NoNewline
Add-Content -Path "version.json" -Value "`n"

# --- Commit y push ---
git add version.json
git commit -m "v$version"
git push

Write-Host "Release v$version publicado correctamente." -ForegroundColor Green
