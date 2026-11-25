# Minot Datei für unser Node mit dlg_cut.
# In VSCode kann man für Syntax Highlighting und auch weitere Funktionalität die Erweiterung "Minot" installieren.
# Wir geben aber in der Übung nur Anweisungen für die TUI.

#--- init
_bag.{
  lidar.{
    _topic = "/ouster/points"
    _type = Any
    _short = "l"
  }
  imu.{
    _topic = "/ouster/imu"
    _type = Any
    _short = "i"
  }
  odom.{
    _topic = "/odom"
    _type = Any
    _short = "o"
  }
  static_tf.{
    _topic = "/tf_static"
    _type = Any
    _short = "tf"
  }
}

# Bagfile direkt über Marina. Alternativ einen Pfad angeben, entweder relativ
# zu dieser Datei (Start mit "./") oder absolut (Start "/")
reset! dlg_feldtage_24:cut
#---

#--- packet
pf! [i, o, l, tf], l 1
pf! l 1
#---
