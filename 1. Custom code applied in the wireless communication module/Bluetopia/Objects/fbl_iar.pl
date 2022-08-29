#!/usr/bin/perl
#
# File:        fbl_iar.pl
# Author:      Tim Cook
# Description: Builds Flexible Build Library for IAR
# Usage:       fbl_iar.pl [--sppserver] [--sppclient] [--sdpserver] [--sdpclient]
#                         [--lemaster] [--leslave] [--gattserver] [--gattclient]
#                         [--hidhost]  [--hiddevice] [--sco] [--largemtu] [--help]
#
use strict;
use Getopt::Long;

my $SPPServer;
my $SPPClient;
my $SDPServer;
my $SDPClient;
my $LEMaster;
my $LESlave;
my $GATTServer;
my $GATTClient;
my $LargeMTU;
my $FolderPrefix;
my $HIDHost;
my $HIDDevice;
my $SCO;
my @Objects;
my $BluetopiaLibrary;
my @GATTObjects;
my $GATTLibrary = 'libSS1BTGAT.a';
my @HIDObjects;
my $HIDLibrary  = 'libSS1BTHID.a';
my $BuildOutput;
my $Help;

print "\n";
print "############################################################################\n";
print "Running Flexible Build Library Utility.  IAR Archiver (xar) must be in the \npath variable.\n";
print "############################################################################\n";
print "\n";

#-------------------------------------------------------------------------
# Parse the command line switches
#-------------------------------------------------------------------------
GetOptions('sppserver'  => \$SPPServer,
           'sppclient'  => \$SPPClient,
           'sdpserver'  => \$SDPServer,
           'sdpclient'  => \$SDPClient,
           'lemaster'   => \$LEMaster,
           'leslave'    => \$LESlave,
           'gattserver' => \$GATTServer,
           'gattclient' => \$GATTClient,
           'hidhost'    => \$HIDHost,
           'hiddevice'  => \$HIDDevice,
           'largemtu'   => \$LargeMTU,
           'sco'        => \$SCO,
           'help'       => \$Help
           );

#-------------------------------------------------------------------------
# Display Help string if requested
#-------------------------------------------------------------------------
if($Help)
{
print <<HELP;
   fbl_iar.pl options (Note all options are optional):
      --sppserver  - Support SPP Server Role
      --sppclient  - Support SPP Client Role
      --sdpserver  - Support SDP Server Role
      --sdpclient  - Support SDP Client Role
      --lemaster   - Support LE Master Role
      --leslave    - Support LE Slave Role
      --gattserver - Support GATT Server Role
      --gattclient - Support GATT Client Role
      --hidhost    - Support HID Host Role
      --hiddevice  - Support HID Device Role
      --largemtu   - Large MTU supported
      --sco        - Support SCO (Audio)
HELP

   exit(0);
}

#-------------------------------------------------------------------------
# Determine the folder prefix
#-------------------------------------------------------------------------
if($LargeMTU)
{
   $FolderPrefix = "LargeMTU";
}
else
{
   $FolderPrefix = "DefaultMTU";
}

#-------------------------------------------------------------------------
# Note if we are including SPP Server support we must have SDP Server support
#-------------------------------------------------------------------------
if($SPPServer == 1)
{
   $SDPServer = 1;
}

#-------------------------------------------------------------------------
# Add the common objects
#-------------------------------------------------------------------------
push(@Objects, "IAR\\$FolderPrefix\\BSC.r43");

#-------------------------------------------------------------------------
# Determine which SPP and RFCOMM objects are to be included
#-------------------------------------------------------------------------
if(($SPPServer == 1) && ($SPPClient == 1))
{
#   print "Including SPP Server and Client support.\n";
   print "SPP Support:   Server/Client.\n";

   push(@Objects, "IAR\\$FolderPrefix\\RFCOMM_SC.r43");
   push(@Objects, "IAR\\$FolderPrefix\\SPP_SC.r43");
}
elsif(($SPPServer == 1) && ($SPPClient == 0))
{
#   print "Including SPP Server support.\n";
   print "SPP Support:   Server.\n";

   push(@Objects, "IAR\\$FolderPrefix\\RFCOMM_S.r43");
   push(@Objects, "IAR\\$FolderPrefix\\SPP_S.r43");
}
elsif(($SPPServer == 0) && ($SPPClient == 1))
{
#   print "Including SPP Client support.\n";
   print "SPP Support:   Client.\n";

   push(@Objects, "IAR\\$FolderPrefix\\RFCOMM_C.r43");
   push(@Objects, "IAR\\$FolderPrefix\\SPP_C.r43");
}
else
{
#   print "Not including SPP support.\n";
   print "SPP Support:   None.\n";

   push(@Objects, "IAR\\$FolderPrefix\\RFCOMM_Empty.r43");
   push(@Objects, "IAR\\$FolderPrefix\\SPP_Empty.r43");
}

#-------------------------------------------------------------------------
# Determine which SDP Object is to be included
#-------------------------------------------------------------------------
if($SDPClient == 1)
{
#   print "Including SDP Server and Client support.\n";
   print "SDP Support:   Server/Client.\n";

   push(@Objects, "IAR\\$FolderPrefix\\SDP_SC.r43");
}
else
{
#   print "Including SDP Server support.\n";
   print "SDP Support:   Server.\n";

   push(@Objects, "IAR\\$FolderPrefix\\SDP_S.r43");
}

#-------------------------------------------------------------------------
# Determine which HCI Object to include
#-------------------------------------------------------------------------
if(($SCO == 1) && ($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including SCO and Low Energy Master and Slave Role support in HCI.\n";
   print "HCI Support:   SCO,LE Master/Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_SCO_MS.r43");
}
elsif(($SCO == 1) && ($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including SCO and Low Energy Master Role support in HCI.\n";
   print "HCI Support:   SCO,LE Master.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_SCO_M.r43");
}
elsif(($SCO == 1) && ($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including SCO and Low Energy Slave Role support in HCI.\n";
   print "HCI Support:   SCO,LE Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_SCO_S.r43");
}
elsif(($SCO == 1) && ($LEMaster == 0) && ($LESlave == 0))
{
#   print "Including SCO support in HCI.\n";
   print "HCI Support:   SCO.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_SCO.r43");
}
elsif(($SCO == 0) && ($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including Low Energy Master and Slave Role support in HCI.\n";
   print "HCI Support:   LE Master/Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_NOSCO_MS.r43");
}
elsif(($SCO == 0) && ($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including Low Energy Master Role support in HCI.\n";
   print "HCI Support:   LE Master.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_NOSCO_M.r43");
}
elsif(($SCO == 0) && ($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including Low Energy Slave Role support in HCI.\n";
   print "HCI Support:   LE Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_NOSCO_S.r43");
}
else
{
#   print "Including HCI with no SCO or LE support.\n";
   print "HCI Support:   Standard HCI Only.\n";

   push(@Objects, "IAR\\$FolderPrefix\\HCI_NOSCO.r43");
}

#-------------------------------------------------------------------------
# Determine which GAP Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) || ($LESlave == 1))
{
   push(@Objects, "IAR\\$FolderPrefix\\GAP_LE.r43");
}
else
{
   push(@Objects, "IAR\\$FolderPrefix\\GAP_NoLE.r43");
}

#-------------------------------------------------------------------------
# Determine which GAPLE Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including Low Energy Master and Slave Role support.\n";
   print "GAP Support:   BR/EDR, LE Master/Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\GAPLE_MS.r43");
}
elsif(($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including Low Energy Master Role support.\n";
   print "GAP Support:   BR/EDR, LE Master.\n";

   push(@Objects, "IAR\\$FolderPrefix\\GAPLE_M.r43");
}
elsif(($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including Low Energy Slave Role support.\n";
   print "GAP Support:   BR/EDR, LE Slave.\n";

   push(@Objects, "IAR\\$FolderPrefix\\GAPLE_S.r43");
}
else
{
   print "GAP Support:   BR/EDR.\n";

   push(@Objects, "IAR\\$FolderPrefix\\GAPLE_Empty.r43");
}

#-------------------------------------------------------------------------
# Determine which SCO Object to include
#-------------------------------------------------------------------------
if($SCO)
{
#   print "Including SCO support.\n";
   print "SCO Support:   Yes.\n";

   push(@Objects, "IAR\\$FolderPrefix\\SCO.r43");
}
else
{
#   print "Not including SCO support.\n";
   print "SCO Support:   No.\n";

   push(@Objects, "IAR\\$FolderPrefix\\SCO_Empty.r43");
}

#-------------------------------------------------------------------------
# Determine which L2CAP Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) || ($LESlave == 1))
{
   print "L2CAP Support: BR/EDR, LE.\n";

   push(@Objects, "IAR\\$FolderPrefix\\L2CAP_LE.r43");
}
else
{
   print "L2CAP Support: BR/EDR.\n";

   push(@Objects, "IAR\\$FolderPrefix\\L2CAP_NoLE.r43");
}

#-------------------------------------------------------------------------
# Determine which GATT Object to include
#-------------------------------------------------------------------------
if(($GATTServer == 1) && ($GATTClient == 1))
{
#   print "Including GATT Server and Client support.\n";
   print "GATT Support:  Server/Client.\n";

   push(@GATTObjects, "IAR\\$FolderPrefix\\GATT_SC.r43");
}
elsif(($GATTServer == 1) && ($GATTClient == 0))
{
#   print "Including GATT Server support.\n";
   print "GATT Support:  Server.\n";

   push(@GATTObjects, "IAR\\$FolderPrefix\\GATT_S.r43");
}
elsif(($GATTServer == 0) && ($GATTClient == 1))
{
#   print "Including GATT Client support.\n";
   print "GATT Support:  Client.\n";

   push(@GATTObjects, "IAR\\$FolderPrefix\\GATT_C.r43");
}

#-------------------------------------------------------------------------
# Determine which HID Object to include
#-------------------------------------------------------------------------
if(($HIDHost == 1) && ($HIDDevice == 1))
{
   print "HID Support:   Host/Device.\n";

   push(@HIDObjects, "IAR\\$FolderPrefix\\HID_HD.r43");
}
elsif(($HIDHost == 1) && ($HIDDevice == 0))
{
   print "HID Support:   Host.\n";

   push(@HIDObjects, "IAR\\$FolderPrefix\\HID_H.r43");
}
elsif(($HIDHost == 0) && ($HIDDevice == 1))
{
   print "HID Support:   Device.\n";

   push(@HIDObjects, "IAR\\$FolderPrefix\\HID_D.r43");
}

#-------------------------------------------------------------------------
# Determine what to call the library
#-------------------------------------------------------------------------
if(($LEMaster == 1) || ($LESlave == 1))
{
   $BluetopiaLibrary = "libBluetopia_LE.a";
}
else
{
   $BluetopiaLibrary = "libBluetopia.a";
}

#-------------------------------------------------------------------------
# Create the library with the specified files
#-------------------------------------------------------------------------
unlink($BluetopiaLibrary);
$BuildOutput = `xar @Objects -o $BluetopiaLibrary`;

print $BuildOutput;

#-------------------------------------------------------------------------
# Create the GATT library with the specified files if requested
#-------------------------------------------------------------------------
if(scalar(@GATTObjects) > 0)
{
   unlink($GATTLibrary);
   $BuildOutput = `xar @GATTObjects -o $GATTLibrary`;

   print $BuildOutput;
}

#-------------------------------------------------------------------------
# Create the HID library with the specified files if requested
#-------------------------------------------------------------------------
if(scalar(@HIDObjects) > 0)
{
   unlink($HIDLibrary);
   $BuildOutput = `xar @HIDObjects -o $HIDLibrary`;

   print $BuildOutput;
}
