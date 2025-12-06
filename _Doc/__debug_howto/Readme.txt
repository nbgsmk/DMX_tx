
main.c
------
- Pronaci sekciju user code begin 0 i dodati override ispod.
- Zatim podesiti debug config prema slici 1
- Zatim startovati debug seansu, pauzirati ili sacekati breakpoint. Bez toga, dalji koraci ne mogu da se podese!
- Dok je aplikacija pauzirana, preci u debug perspective, prikazati show view -> SWV ..graph, SWV...data console, SWV...statistical profiling
- Na bilo kom od tih prozora kliknuti ikonicu 'settings' i podesiti prema slici 2.
- Pokrenuti "record" za SWV (crveno dugme odmah pored settings)
- Resume debug aplikacije -> swo varijable se prikazuju uzivo!
- Kad se ponovo pauzira aplikacija, azurirace se view->statistical profiling za svaki task, sistemski poziv i slicno! Fenomenalna stvar! (ne menja se u toku rada vec samo prilikom pauze)
---------------------------------------------------------------------------

/* USER CODE BEGIN 0 */

// ***************************
// ***************************
// SWO - Serial Wire Viewer override for printf(..)
// use like this printf("chan %d \n", itm0)
// ***************************
// ***************************
int itm0 = 0;		// must be global vars
int itm1 = 0;
char itmC0[256];
char itmC1[256];
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

---------------------------------------------------------------------------

Note!
// ***************************
// ***************************
// Verzija 2 - advanced!
//
// https://youtu.be/j-GaEZKrkbQ?t=529
// ***************************
// ***************************

Postoji funkcija fprintf(..) koja prima argument filedescriptor.
Default filedescriptori su stdin=0, stdout=0/1, stderr=2.
printf -> default filedescriptor je 0;

Pa mozemo overridovati ovako

..... ....  _write(int fd, int file, char *ptr, int len) {
	if (fd == 1) {							// to stdout
		HAL_StatusTypeDef hstat;
		hstat = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (hstat == HAL_OK) {
			return len;
		} else {
			return -1;
		}
	} else if (fd == 2) {					// to stderr
		for (int i = 0; i < len; i++) {
			ITM_SendChar(ptr[i]);			// core_cm4.h
		}
		return len;
	} else {
		return -1;	// zasto ovo??
	}
	return -1;
	}
}

