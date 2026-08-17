Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip = [System.IO.Compression.ZipFile]::OpenRead("C:\Ardupilot_Test\ardupilot-master\Standards\Requirements_Standards_v2.docx")
$entry = $zip.Entries | Where-Object { $_.FullName -eq "word/document.xml" }
$stream = $entry.Open()
$reader = New-Object System.IO.StreamReader($stream)
$xml = $reader.ReadToEnd()
$reader.Close()
$zip.Dispose()
$text = $xml -replace '<[^>]+>', ' '
$text = $text -replace '\s+', ' '
$text.Trim() | Out-File -FilePath "C:\Ardupilot_Test\ardupilot-master\Standards\extracted.txt" -Encoding UTF8
Write-Host "Done"
