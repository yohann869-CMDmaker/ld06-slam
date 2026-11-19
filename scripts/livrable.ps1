# Fabrique le .zip du rendu (énoncé, « Livrable ») à côté du dossier du dépôt.
#
# Version légère (par défaut) : pour la plateforme de rendu, limitée à 10 Mo.
#   - LIEN_DEPOT.txt : lien du dépôt GitHub public, qui porte tout l'historique des commits
#     (l'énoncé demande « un lien vers votre dépôt Git accessible à l'évaluateur, OU une copie du
#     dépôt incluant son historique ») et les vidéos du robot ;
#   - ld06-slam/ : les outils, les données, les comptes rendus et les figures (fichiers suivis par
#     Git), sans le dossier .git ni les vidéos ;
#   - SYNTHESE.md : la synthèse personnelle ;
#   - LISEZMOI.txt : comment tester les outils.
# Version complète (-Complet) : en plus, le dossier .git (historique) et les vidéos (~39 Mo).
#
# Usage (PowerShell, depuis le dossier ld06-slam) :  .\scripts\livrable.ps1   ou   .\scripts\livrable.ps1 -Complet
# Tout doit être commité avant : la copie ne prend que ce qui est dans Git.
param([switch]$Complet, [double]$MaxMo = 10)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$parent = Split-Path -Parent $repo
$name = 'ZOUNGRANA_Yohan_Cartographie_Localisation'
if ($Complet) { $name += '_complet' }
$work = Join-Path $parent $name
$zip = Join-Path $parent "$name.zip"
$url = 'https://github.com/yohann869-CMDmaker/ld06-slam'

$dirty = git -C $repo status --porcelain --untracked-files=no
if ($dirty) { Write-Warning "Modifications non commitées : elles ne seront PAS dans le zip.`n$dirty" }

if (Test-Path $work) { Remove-Item -Recurse -Force $work }
if (Test-Path $zip) { Remove-Item -Force $zip }
New-Item -ItemType Directory $work | Out-Null

$copy = Join-Path $work 'ld06-slam'
git clone --quiet $repo $copy
git -C $copy remote set-url origin "$url.git"
$commits = git -C $repo rev-list --count HEAD
$videos = Get-ChildItem (Join-Path $copy 'docs/media') -Filter *.mp4 -ErrorAction SilentlyContinue
if (-not $Complet) {
  Remove-Item -Recurse -Force (Join-Path $copy '.git')
  $videos | Remove-Item -Force
}

$videoLinks = ($videos | ForEach-Object { "  $url/blob/main/docs/media/$($_.Name)" }) -join "`n"
$historyLine = if ($Complet) { "Le dossier ld06-slam/ de ce zip en est une copie complète, historique compris (git log)." } else { "Le dossier ld06-slam/ de ce zip contient les outils, les données et les comptes rendus (sans le dossier .git ni les vidéos, pour respecter la limite de taille du dépôt du devoir). L'historique complet des commits ($commits commits) est sur GitHub : $url/commits/main" }
Set-Content -Encoding utf8 (Join-Path $work 'LIEN_DEPOT.txt') @"
Dépôt Git (public, avec tout l'historique des commits) :
$url

$historyLine

Vidéos du robot piloté par câble pendant le SLAM en direct :
$videoLinks
"@

# synthèse à la racine du zip : liens et images réécrits vers ld06-slam/docs/
$synth = Get-Content -Raw -Encoding utf8 (Join-Path $repo 'docs/synthese.md')
$synth = [regex]::Replace($synth, '\]\((?!https?:|#|/)([^)]+)\)', { param($m) "](ld06-slam/docs/$($m.Groups[1].Value))" })
if (-not $Complet) { $synth = $synth -replace '\]\(ld06-slam/docs/media/([^)]+\.mp4)\)', "]($url/blob/main/docs/media/`$1)" }
Set-Content -Encoding utf8 (Join-Path $work 'SYNTHESE.md') $synth

Set-Content -Encoding utf8 (Join-Path $work 'LISEZMOI.txt') @"
Cartographie et localisation 2D avec le lidar LD06 - Yohan Zoungrana

Contenu
  LIEN_DEPOT.txt  lien du dépôt GitHub (historique complet des commits, vidéos du robot)
  SYNTHESE.md     synthèse personnelle (par outil, puis ce que je retiens du module)
  ld06-slam/      les outils, les données, les comptes rendus et les figures

Tester (Node.js 22 ou plus, aucune dépendance)
  cd ld06-slam
  node serve.mjs              puis ouvrir http://localhost:8080/
  node --test "test/*.test.js"

ld06-slam/README.md : démarrage en 5 minutes, tableau « chaque demande de l'énoncé -> où la trouver »,
ajouts au-delà de l'énoncé, programmes du robot, scripts qui régénèrent tous les chiffres.
Le même README s'affiche sur la page GitHub du dépôt.
"@

# ZipFile plutôt que Compress-Archive : Compress-Archive saute les dossiers cachés (.git)
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($work, $zip, [IO.Compression.CompressionLevel]::Optimal, $false)
if ($Complet) {
  $z = [IO.Compression.ZipFile]::OpenRead($zip)
  $ok = [bool]($z.Entries | Where-Object { $_.FullName -like 'ld06-slam/.git/*' } | Select-Object -First 1)
  $z.Dispose()
  if (-not $ok) { throw "Le zip ne contient pas ld06-slam/.git : historique manquant" }
}
$mo = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Host "Livrable : $zip ($mo Mo, $commits commits sur GitHub)"
if (-not $Complet -and $mo -gt $MaxMo) { Write-Warning "Plus lourd que $MaxMo Mo : la plateforme risque de le refuser." }
