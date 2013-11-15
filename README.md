mxs-auart-fiq
=============

High speed i.MX23/28 Linux UART driver

This is a draft Freescale i.MX23/8 application UART (auart) driver that is intended to solve the conflict
between the short, sixteen-byte PrimeCell UART FIFO's, high-speed traffic (57600B or higher), and Linux's non-preemptive
scheduler.

The driver can be compiled to use either the UART interrupts as the FIQ interrupt, or the timrot2 timer interrupt as the
FIQ interrupt. Either solution provides reliable interrupt handling.

The current mxs-auart-fiq.c code compiles and runs in linux-3.10.5/drivers/tty/serial.

The device name is /dev/ttyFiq<n>

Please send complaints, comments, and requests for support to Jonathan Ben-Avraham <yba@tkos.co.il>.

This work was financed by Goji Food Solutions Ltd., by FriskyDSP Ltd., and by Jonathan Ben-Avraham
