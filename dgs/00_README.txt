Code in this directory is intended to be mechanism only. Therefore it should not include header files of the main plugin.

It provides code to manage placed DGS with appropriate guidance signals.

The purpose of classes is:

dgs::AptAirport
    This is essentially an extract of all apt.dat files in order to get a list of stand coordinates for airports.
    It is built once during plugin start and kept forever. As we get around 6000 airports with 120,000 stands
    it aims to have a small footprint.

dgs::Airport
    Is an AptAirport enriched with status and geometry information for guidance. It it build once a
    departure or arrival airport is established by the main plugin.
    It does not provide policy code that determines what DGS is placed where. It just manages and drives
    placed DGS displays.

dgs::DGS
    Abstract base class for DGS models.

In the main plugin:

AdgsAirport, OsAirport
    Derived from dgs::Airport.
    These contain policy code what DGS to be placed where, e.g. learn placed objects in the scenery or actively
    place DGS objects in default airports.

