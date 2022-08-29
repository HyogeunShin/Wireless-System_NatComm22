#!/usr/bin/perl
#
# File:        fbl_ccs.pl
# Author:      Tim Cook
# Description: Builds Flexible Build Library for CCS
# Usage:       fbl_ccs.pl [--sppserver] [--sppclient] [--sdpserver] [--sdpclient]
#                         [--lemaster] [--leslave] [--gattserver] [--gattclient]
#                         [--hidhost]  [--hiddevice] [--sco] [--largemtu] [--eabi] [--help]
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
my $EABI;
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
print "Running Flexible Build Library Utility.  CCS Archiver (ar430) must be in the \npath variable.\n";
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
           'eabi'       => \$EABI,
           'help'       => \$Help
           );

#-------------------------------------------------------------------------
# Display Help string if requested
#-------------------------------------------------------------------------
if($Help)
{
print <<HELP;
   fbl_ccs.pl options (Note all options are optional):
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
      --eabi       - Specify EABI format (default is COFFABI)
HELP

   exit(0);
}

#-------------------------------------------------------------------------
# Determine the folder prefix
#-------------------------------------------------------------------------
if($EABI)
{
   $FolderPrefix = "eabi\\";
}
else
{
   $FolderPrefix = "coffabi\\";
}

if($LargeMTU)
{
   $FolderPrefix = $FolderPrefix . "LargeMTU";
}
else
{
   $FolderPrefix = $FolderPrefix . "DefaultMTU";
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
push(@Objects, "CCS\\$FolderPrefix\\BSC.obj");

#-------------------------------------------------------------------------
# Determine which SPP and RFCOMM objects are to be included
#-------------------------------------------------------------------------
if(($SPPServer == 1) && ($SPPClient == 1))
{
#   print "Including SPP Server and Client support.\n";
   print "SPP Support:   Server/Client.\n";

   push(@Objects, "CCS\\$FolderPrefix\\RFCOMM_SC.obj");
   push(@Objects, "CCS\\$FolderPrefix\\SPP_SC.obj");
}
elsif(($SPPServer == 1) && ($SPPClient == 0))
{
#   print "Including SPP Server support.\n";
   print "SPP Support:   Server.\n";

   push(@Objects, "CCS\\$FolderPrefix\\RFCOMM_S.obj");
   push(@Objects, "CCS\\$FolderPrefix\\SPP_S.obj");
}
elsif(($SPPServer == 0) && ($SPPClient == 1))
{
#   print "Including SPP Client support.\n";
   print "SPP Support:   Client.\n";

   push(@Objects, "CCS\\$FolderPrefix\\RFCOMM_C.obj");
   push(@Objects, "CCS\\$FolderPrefix\\SPP_C.obj");
}
else
{
#   print "Not including SPP support.\n";
   print "SPP Support:   None.\n";

   push(@Objects, "CCS\\$FolderPrefix\\RFCOMM_Empty.obj");
   push(@Objects, "CCS\\$FolderPrefix\\SPP_Empty.obj");
}

#-------------------------------------------------------------------------
# Determine which SDP Object is to be included
#-------------------------------------------------------------------------
if($SDPClient == 1)
{
#   print "Including SDP Server and Client support.\n";
   print "SDP Support:   Server/Client.\n";

   push(@Objects, "CCS\\$FolderPrefix\\SDP_SC.obj");
}
else
{
#   print "Including SDP Server support.\n";
   print "SDP Support:   Server.\n";

   push(@Objects, "CCS\\$FolderPrefix\\SDP_S.obj");
}

#-------------------------------------------------------------------------
# Determine which HCI Object to include
#-------------------------------------------------------------------------
if(($SCO == 1) && ($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including SCO and Low Energy Master and Slave Role support in HCI.\n";
   print "HCI Support:   SCO,LE Master/Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_SCO_MS.obj");
}
elsif(($SCO == 1) && ($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including SCO and Low Energy Master Role support in HCI.\n";
   print "HCI Support:   SCO,LE Master.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_SCO_M.obj");
}
elsif(($SCO == 1) && ($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including SCO and Low Energy Slave Role support in HCI.\n";
   print "HCI Support:   SCO,LE Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_SCO_S.obj");
}
elsif(($SCO == 1) && ($LEMaster == 0) && ($LESlave == 0))
{
#   print "Including SCO support in HCI.\n";
   print "HCI Support:   SCO.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_SCO.obj");
}
elsif(($SCO == 0) && ($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including Low Energy Master and Slave Role support in HCI.\n";
   print "HCI Support:   LE Master/Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_NOSCO_MS.obj");
}
elsif(($SCO == 0) && ($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including Low Energy Master Role support in HCI.\n";
   print "HCI Support:   LE Master.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_NOSCO_M.obj");
}
elsif(($SCO == 0) && ($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including Low Energy Slave Role support in HCI.\n";
   print "HCI Support:   LE Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_NOSCO_S.obj");
}
else
{
#   print "Including HCI with no SCO or LE support.\n";
   print "HCI Support:   Standard HCI Only.\n";

   push(@Objects, "CCS\\$FolderPrefix\\HCI_NOSCO.obj");
}

#-------------------------------------------------------------------------
# Determine which GAP Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) || ($LESlave == 1))
{
   push(@Objects, "CCS\\$FolderPrefix\\GAP_LE.obj");
}
else
{
   push(@Objects, "CCS\\$FolderPrefix\\GAP_NoLE.obj");
}

#-------------------------------------------------------------------------
# Determine which GAPLE Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) && ($LESlave == 1))
{
#   print "Including Low Energy Master and Slave Role support.\n";
   print "GAP Support:   BR/EDR, LE Master/Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\GAPLE_MS.obj");
}
elsif(($LEMaster == 1) && ($LESlave == 0))
{
#   print "Including Low Energy Master Role support.\n";
   print "GAP Support:   BR/EDR, LE Master.\n";

   push(@Objects, "CCS\\$FolderPrefix\\GAPLE_M.obj");
}
elsif(($LEMaster == 0) && ($LESlave == 1))
{
#   print "Including Low Energy Slave Role support.\n";
   print "GAP Support:   BR/EDR, LE Slave.\n";

   push(@Objects, "CCS\\$FolderPrefix\\GAPLE_S.obj");
}
else
{
   print "GAP Support:   BR/EDR.\n";

   push(@Objects, "CCS\\$FolderPrefix\\GAPLE_Empty.obj");
}

#-------------------------------------------------------------------------
# Determine which SCO Object to include
#-------------------------------------------------------------------------
if($SCO)
{
#   print "Including SCO support.\n";
   print "SCO Support:   Yes.\n";

   push(@Objects, "CCS\\$FolderPrefix\\SCO.obj");
}
else
{
#   print "Not including SCO support.\n";
   print "SCO Support:   No.\n";

   push(@Objects, "CCS\\$FolderPrefix\\SCO_Empty.obj");
}

#-------------------------------------------------------------------------
# Determine which L2CAP Object to include
#-------------------------------------------------------------------------
if(($LEMaster == 1) || ($LESlave == 1))
{
   print "L2CAP Support: BR/EDR, LE.\n";
   push(@Objects, "CCS\\$FolderPrefix\\L2CAP_LE.obj");
}
else
{
   print "L2CAP Support: BR/EDR.\n";
   push(@Objects, "CCS\\$FolderPrefix\\L2CAP_NoLE.obj");
}

#-------------------------------------------------------------------------
# Determine which GATT Object to include
#-------------------------------------------------------------------------
if(($GATTServer == 1) && ($GATTClient == 1))
{
#   print "Including GATT Server and Client support.\n";
   print "GATT Support:  Server/Client.\n";

   push(@GATTObjects, "CCS\\$FolderPrefix\\GATT_SC.obj");
}
elsif(($GATTServer == 1) && ($GATTClient == 0))
{
#   print "Including GATT Server support.\n";
   print "GATT Support:  Server.\n";

   push(@GATTObjects, "CCS\\$FolderPrefix\\GATT_S.obj");
}
elsif(($GATTServer == 0) && ($GATTClient == 1))
{
#   print "Including GATT Client support.\n";
   print "GATT Support:  Client.\n";

   push(@GATTObjects, "CCS\\$FolderPrefix\\GATT_C.obj");
}

#-------------------------------------------------------------------------
# Determine which HID Object to include
#-------------------------------------------------------------------------
if(($HIDHost == 1) && ($HIDDevice == 1))
{
   print "HID Support:   Host/Device.\n";

   push(@HIDObjects, "CCS\\$FolderPrefix\\HID_HD.obj");
}
elsif(($HIDHost == 1) && ($HIDDevice == 0))
{
   print "HID Support:   Host.\n";

   push(@HIDObjects, "CCS\\$FolderPrefix\\HID_H.obj");
}
elsif(($HIDHost == 0) && ($HIDDevice == 1))
{
   print "HID Support:   Device.\n";

   push(@HIDObjects, "CCS\\$FolderPrefix\\HID_D.obj");
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
$BuildOutput = `ar430 r $BluetopiaLibrary @Objects`;

print $BuildOutput;

#-------------------------------------------------------------------------
# Create the GATT library with the specified files if requested
#-------------------------------------------------------------------------
if(scalar(@GATTObjects) > 0)
{
   unlink($GATTLibrary);
   $BuildOutput = `ar430 r $GATTLibrary @GATTObjects`;

   print $BuildOutput;
}

#-------------------------------------------------------------------------
# Create the HID library with the specified files if requested
#-------------------------------------------------------------------------
if(scalar(@HIDObjects) > 0)
{
   unlink($HIDLibrary);
   $BuildOutput = `ar430 r $HIDLibrary @HIDObjects`;

   print $BuildOutput;
}
