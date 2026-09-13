This repository contains a tool for flashing custom firmware onto G2 smart
glasses, plus a custom firmware designed with Faceclaw in mind.

DEVELOPING YOUR OWN FIRMWARE MODS IS MUCH RISKIER THAN INSTALLING FIRMWARE THAT
HAS ALREADY BEEN TESTED. If you are considering writing firmware mods and you
can complete a project using only phone-side changes, you are almost certainly
better off doing it that way. It is a bad idea to work in this codebase using a
non-frontier language model, or without a proper coding harness, or a
nontechnical user steering.

Changes that involve overwriting parts of the existing firmware are more risky
than changes to the existing C extension files. Changes that run or affect
behavior prior to receiving any custom messages are especially risky, because
if you make the glasses crash on boot, they won't be able to accept an OTA
update. Rebasing the mods onto a new base firmware is an especially hazardous
operation, since it requires updating every firmware offset correctly and not
missing any.

