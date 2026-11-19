# Fabrique le .zip du rendu (énoncé, « Livrable ») à côté du dossier du dépôt :
#   - LIEN_DEPOT.txt : lien du dépôt GitHub public ;
#   - ld06-slam/ : copie du dépôt AVEC son historique Git (clone local : seuls les fichiers suivis par
#     Git, donc ni out/ ni les fichiers de travail non suivis) ;
#   - SYNTHESE.md : la synthèse personnelle (copie de docs/synthese.md) ;
#   - LISEZMOI.txt : comment tester les outils.
# Usage (PowerShell, depuis le dossier ld06-slam) :  .\scripts\livrable.ps1
# Tout doit être commité avant : le clone ne prend que ce qui est dans Git.
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$parent = Split-Path -Parent $repo
$name = 'ZOUNGRANA_Yohan_Cartographie_Localisation'
$work = Join-Path $parent $name
$zip = Join-Path $parent "$name.zip"
$url = 'https://github.com/yohann869-CMDmaker/ld06-slam'

$dirty = git -C $repo status --porcelain --untracked-files=no
if ($dirty) { Write-Warning "Modifications non commitées : elles ne seront PAS dans le zip.`n$dirty" }

if (Test-Path $work) { Remove-Item -Recurse -Force $work }
if (Test-Path $zip) { Remove-Item -Force $zip }
New-Item -ItemType Directory $work | Out-Null

git clone --quiet $repo (Join-Path $work 'ld06-slam')
git -C (Join-Path $work 'ld06-slam') remote set-url origin "$url.git"

Set-Content -Encoding utf8 (Join-Path $work 'LIEN_DEPOT.txt') @"
Dépôt Git (public, avec tout l'historique des commits) :
$url

Le dossier ld06-slam/ de ce zip en est une copie complète, historique compris (git log).
"@

Copy-Item (Join-Path $repo 'docs/synthese.md') (Join-Path $work 'SYNTHESE.md')

Set-Content -Encoding utf8 (Join-Path $work 'LISEZMOI.txt') @"
Cartographie et localisation 2D avec le lidar LD06 - Yohan Zoungrana

Contenu
  LIEN_DEPOT.txt  lien du dépôt GitHub
  SYNTHESE.md     synthèse personnelle (par outil, puis ce que je retiens du module)
  ld06-slam/      les outils, les données, les comptes rendus ; historique Git complet

Tester (Node.js 22 ou plus, aucune dépendance)
  cd ld06-slam
  node serve.mjs              puis ouvrir http://localhost:8080/
  node --test "test/*.test.js"

ld06-slam/README.md : démarrage en 5 minutes, tableau « chaque demande de l'énoncé -> où la trouver »,
ajouts au-delà de l'énoncé, programmes du robot, scripts qui régénèrent tous les chiffres.
"@

# ZipFile plutôt que Compress-Archive : Compress-Archive saute les dossiers cachés, donc .git (l'historique)
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::CreateFromDirectory($work, $zip, [IO.Compression.CompressionLevel]::Optimal, $false)
$hasGit = [IO.Compression.ZipFile]::OpenRead($zip)
$ok = [bool]($hasGit.Entries | Where-Object { $_.FullName -like 'ld06-slam/.git/*' -or $_.FullName -like 'ld06-slam\.git\*' } | Select-Object -First 1)
$hasGit.Dispose()
if (-not $ok) { throw "Le zip ne contient pas ld06-slam/.git : historique manquant" }
$mo = [math]::Round((Get-Item $zip).Length / 1MB, 1)
$commits = git -C $repo rev-list --count HEAD
Write-Host "Livrable : $zip ($mo Mo, $commits commits dans l'historique)"
