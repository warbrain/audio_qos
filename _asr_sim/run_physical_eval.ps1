# Physical-playback evaluation: capture N clips at several noise levels through
# the real box, then score both UAC channels (L = chip-denoised, R = raw).
#
# The box's noise speakers (Out3/Out4) are not connected yet, so noise is mixed
# into the artificial-mouth channel with --noise-mouth. The signal still travels
# the full physical path: mouth -> mic -> denoise chip -> ESP-NOW -> UAC.
$ErrorActionPreference = 'Continue'
$env:PYTHONIOENCODING = 'utf-8'

$py     = 'D:\App\cppApp\audio_qos\_praenv\Scripts\python.exe'
$root   = 'D:\App\cppApp\audio_qos\_asr_sim'
$out    = Join-Path $root 'physical_eval'
$capture = Join-Path $root 'box_capture.py'
$score   = Join-Path $root 'score_capture.py'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$log = Join-Path $out 'results.txt'
"物理采集评测  $(Get-Date -Format 'yyyy-MM-dd HH:mm')" | Set-Content -LiteralPath $log -Encoding utf8

# 40 clips from the heldout test set
$mk = @'
import json, os
root=r'D:\App\rsApp\voicebox\data\datasets\qwen3-asr-strict-v5'
out=r'D:\App\cppApp\audio_qos\_asr_sim\physical_eval'
mf=json.load(open(os.path.join(root,'heldout-manifest.strict-v5.json'),encoding='utf-8'))
items=[]
for c in mf['recordings'][:40]:
    p=os.path.join(root, c['audio'].replace('/',os.sep))
    items.append({'id': os.path.splitext(os.path.basename(p))[0], 'audio': p,
                  'speaker': c.get('speaker',''), 'reference': c['reference']})
json.dump(items, open(os.path.join(out,'clips40.json'),'w',encoding='utf-8'), ensure_ascii=False, indent=1)
print('clips:', len(items))
'@
$mkPath = Join-Path $out 'mkmanifest.py'
Set-Content -LiteralPath $mkPath -Value $mk -Encoding UTF8
& $py $mkPath

$clips = Join-Path $out 'clips40.json'
$conditions = @(
    @{ name = 'quiet'; extra = '' },
    @{ name = 'snr10'; extra = '--noise-dbfs -40 --noise-mouth' },
    @{ name = 'snr00'; extra = '--noise-dbfs -30 --noise-mouth' }
)

foreach ($c in $conditions) {
    $dir = Join-Path $out $c.name
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
    Write-Host "=== capture $($c.name) ==="
    $capCmd = "`"$py`" `"$capture`" --manifest `"$clips`" --play-dbfs -30 --out `"$dir`" $($c.extra)"
    $rows = Invoke-Expression "& $capCmd" 2>&1 | Select-String -Pattern '^\{'
    Write-Host "    captured rows: $($rows.Count)"
    Write-Host "=== score $($c.name) ==="
    $res = & $py $score --dir $dir --clips $clips 2>&1
    $res | Out-File -LiteralPath (Join-Path $dir 'score.txt') -Encoding utf8
    "--- $($c.name)  (rows=$($rows.Count)) ---" | Out-File -LiteralPath $log -Encoding utf8 -Append
    ($res | Select-String -Pattern '^(降噪 L|原始 R):' | ForEach-Object { $_.Line }) |
        Out-File -LiteralPath $log -Encoding utf8 -Append
}
Write-Host "=== DONE ==="
Get-Content -LiteralPath $log
