# stm32F411_5102
 Polifonijski digitalni sintisajzer na stm32F411_5102 mikrokontroleru <br/>
 Značajke:
 * Polifonija s 8 glasova koji imaju vlastite ovojnice
 * 3 potenciometra za kontroliranje: ukupne glasnoće, omjera mješanja dviju tablica pretraživanja(look up) i granične frekvencije niskopropusnog linearnog filtra
 * LCD ekran koji će pokazivati stanja potenciometra
 * midi
 * ...
![slika_sintic](https://github.com/user-attachments/assets/70538e2f-80b0-4b2f-93c7-5275630b5a62)
<br/>
<br/>
<br/>
# U ovom demo-u koristimo Dexed software kao MIDI out jer ne možemo povezati na klasičan keyboard zato što imamo problem da nam trenutni keyboard konstantno šalje "Timing clock" poruke, ali zato ne šalje NOTE OFF poruku kada je tipka otpuštena.
Prvi potenciometrom se mijenja gain, drugim miješanje sinusnog signala i signala vala pile, a trećim granična frekvencija niskopropusnog filtera <br/>
Treba dodati i ostale parametre na display.
https://github.com/user-attachments/assets/56884afa-5df5-4314-90ca-bbbfe4220cca

