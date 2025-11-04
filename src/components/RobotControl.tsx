import React, { useState, useRef } from "react";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "./ui/card";
import { Button } from "./ui/button";
import { Badge } from "./ui/badge";
import { Alert, AlertDescription } from "./ui/alert";
import {
  Bot,
  WifiOff,
  CheckCircle,
  Send,
  Lightbulb,
} from "lucide-react";
import { toast } from "sonner@2.0.3";

interface RobotStatus {
  isConnected: boolean;
  ledOn: boolean;
}
const gcodeText = `
G90
Contour 1
Color red
G0 X863.00 Y169.00
M3
G1 X863.00 Y181.00
G1 X860.00 Y185.00
...
M5
`; // <-- paste full GCode here

export function RobotControl() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    isConnected: false,
    ledOn: false,
  });

  const [isConnecting, setIsConnecting] = useState(false);
  const [isSendingCommands, setIsSendingCommands] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);

  const hardcodedImage = "backend/color_output/myimage.png"; // ← put your image in /public/images/myimage.png

  // --- Connection handling ---
  const handleConnect = async () => {
    setIsConnecting(true);

    try {
      const ws = new WebSocket("ws://172.20.10.10/ws");
      wsRef.current = ws;

      ws.onopen = () => {
        console.log("WebSocket connected");
        toast.success("Connected to robot");

        ws.send("START_CONNECTION");

        setRobotStatus({
          isConnected: true,
          ledOn: true,
        });

        setIsConnecting(false);
      };

      ws.onerror = (error) => {
        console.error("WebSocket error:", error);
        toast.error("Failed to connect to robot");
        setIsConnecting(false);
      };

      ws.onclose = () => {
        console.log("WebSocket disconnected");
        toast.info("Disconnected from robot");
        setRobotStatus({
          isConnected: false,
          ledOn: false,
        });
      };

      ws.onmessage = (event) => {
        console.log("Message from robot:", event.data);
        toast.info(`Robot: ${event.data}`);
      };
    } catch (error) {
      console.error("Connection error:", error);
      toast.error("Failed to connect to robot");
      setIsConnecting(false);
    }
  };

  const handleDisconnect = () => {
    if (wsRef.current) {
      if (wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send("END_CONNECTION");
      }
      wsRef.current.close();
      wsRef.current = null;
    }

    setRobotStatus({
      isConnected: false,
      ledOn: false,
    });
  };

  const handleSendCommands = async () => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      toast.error("Robot not connected");
      return;
    }

    setIsSendingCommands(true);

    try {
      // 1️⃣ Load the file from public/
      const response = await fetch("backend/color_output/myimage.gcode");
      const gcodeText = await response.text();

      // 2️⃣ Split & filter
      const gcodeLines = gcodeText.split("\n").map(line => line.trim()).filter(line => line && !line.startsWith(";"));
      console.log(`Loaded ${gcodeLines.length} GCode lines`);

      // 3️⃣ Send each line over WebSocket
      for (const line of gcodeLines) {
        wsRef.current.send(line+'\n');
        await new Promise(resolve => setTimeout(resolve, 10));
      }

      toast.success(`Sent ${gcodeLines.length} commands`);
    } catch (err) {
      console.error("Failed to load or send GCode:", err);
      toast.error("Error loading GCode file");
    } finally {
      setIsSendingCommands(false);
    }
  };

  return (
    <div className="space-y-6">
      <div>
        <h1 className="text-3xl">Robot Control</h1>
        <p className="text-muted-foreground">
          Connect and control your DrawBot whiteboard robot
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Connection Panel */}
        <Card className="lg:col-span-1">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Bot className="h-5 w-5" />
              Robot Connection
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            {!robotStatus.isConnected ? (
              <div className="space-y-4">
                <div className="text-center py-4">
                  <WifiOff className="h-8 w-8 mx-auto text-muted-foreground mb-2" />
                  <p className="text-sm text-muted-foreground">
                    No robot connected
                  </p>
                </div>

                {/* LED Status - OFF */}
                <div className="flex items-center justify-center gap-3 p-4 bg-muted rounded-lg">
                  <Lightbulb className="h-6 w-6 text-muted-foreground" />
                  <span className="text-sm">LED: OFF</span>
                </div>

                <div className="space-y-2">
                  <h4 className="font-medium">Robot Connection</h4>
                  <p className="text-sm text-muted-foreground">IP: 172.20.10.10</p>
                  <Button
                    className="w-full"
                    onClick={handleConnect}
                    disabled={isConnecting}
                  >
                    {isConnecting ? "Connecting..." : "Connect"}
                  </Button>
                </div>
              </div>
            ) : (
              <div className="space-y-4">
                <div className="text-center py-4">
                  <div className="flex items-center justify-center gap-2 mb-2">
                    <CheckCircle className="h-8 w-8 text-green-500" />
                  </div>
                  <p className="font-medium">DrawBot-001</p>
                  <Badge
                    variant="outline"
                    className="text-green-600 border-green-600"
                  >
                    Connected
                  </Badge>
                </div>

                {/* LED Status - ON */}
                <div className="flex items-center justify-center gap-3 p-4 bg-green-50 dark:bg-green-950 rounded-lg">
                  <Lightbulb className="h-6 w-6 text-green-600 fill-green-600" />
                  <span className="text-sm font-medium text-green-600">
                    LED: ON
                  </span>
                </div>

                <Button
                  variant="outline"
                  onClick={handleDisconnect}
                  className="w-full"
                >
                  Disconnect
                </Button>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Control Panel */}
        <Card className="lg:col-span-2">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Send className="h-5 w-5" />
              Robot Controls
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            {!robotStatus.isConnected ? (
              <div className="text-center py-12">
                <Bot className="h-12 w-12 mx-auto text-muted-foreground mb-4" />
                <p className="text-muted-foreground">
                  Connect to the robot to access controls
                </p>
              </div>
            ) : (
              <>
                {/* Hardcoded Processed Image */}
                <div className="space-y-4">
                  <h4 className="font-medium">Processed Image Preview</h4>
                  <div className="border rounded-lg overflow-hidden bg-muted">
                    <img
                      src={hardcodedImage}
                      alt="Processed drawing"
                      className="w-full h-auto"
                    />
                  </div>
                </div>

                {/* Send Commands */}
                <div className="space-y-4">
                  <h4 className="font-medium">GCode Commands</h4>
                  <p className="text-sm text-muted-foreground">
                    Send the processed GCode file to the robot
                  </p>

                  <Button
                    onClick={handleSendCommands}
                    disabled={isSendingCommands}
                    size="lg"
                    className="w-full"
                  >
                    {isSendingCommands ? (
                      <span className="animate-pulse">Sending Commands...</span>
                    ) : (
                      <>
                        <Send className="h-5 w-5 mr-2" />
                        Send Commands
                      </>
                    )}
                  </Button>
                </div>
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
