# Layout-4 metadata revocation

Decided 2026-10-23: the V407 (layout 4) keeps the metadata-sector erase as its revocation. The rule, in place of
the generic revocation sentence:

> On layout 1, revocation writes zero to the first descriptor halfword after
> all descriptor, identity, geometry, supply and output-safety checks pass.
> On layout 4, revocation instead erases the complete 4096-byte metadata sector
> at 0x08004000..0x08005000; programming zero over an existing descriptor is
> not permitted. This erase is the first cursor operation. Status mode announces
> its full deadline with BUSY; declared mode covers it in the erase lead. Read back the entire
> metadata sector and accept only the backend's erased halfword values before
> any body-sector erase or body programming. A timeout, supply failure or
> readback failure cancels all remaining operations and reports FAILED; it
> must never advance to the body. An interruption during this first erase
> leaves the body untouched. At restart, only a valid descriptor and matching
> body CRC may boot; otherwise enter recovery and require a fresh ANNOUNCE and
> complete erase (with a fresh SELECT in status mode). The metadata sector participates once in the
> erase cursor, never a second time after revocation. The reserved remainder
> of the metadata sector is erased with it. Body readback, full CRC and the
> descriptor's magic-last commit ordering remain unchanged.

This preserves the implemented ordering. The flash model accepts erased cells
only (including E339), and the transaction/cut/fault matrix checks revocation
before body changes. Real interrupted erase/readback behavior and flash fetch
coherence still require hardware qualification.
