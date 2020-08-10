<?php

$directory = new RecursiveDirectoryIterator('.');
$iterator = new RecursiveIteratorIterator($directory);

foreach ($iterator as $i) {
	if ($i->isFile()) {
		$st = stat($i->getPathname());
		chmod('/workspace/megous.com/orangepi-pc/.eg25g/OpenLinux-SDK/AG35-LLP/ql-ol-kernel/'.$i->getPathname(), $st['mode'] & 0777);
	}
}