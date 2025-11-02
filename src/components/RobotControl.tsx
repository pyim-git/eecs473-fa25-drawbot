import React, { useState, useEffect, useRef } from "react";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "./ui/card";
import { Button } from "./ui/button";
import { Badge } from "./ui/badge";
import { Input } from "./ui/input";
import { Label } from "./ui/label";
import { Textarea } from "./ui/textarea";
import { Separator } from "./ui/separator";
import {
  Bot,
  WifiOff,
  Settings,
  CheckCircle,
  ArrowUp,
  ArrowDown,
  Square,
} from "lucide-react";
import { toast } from "sonner@2.0.3";

interface RobotStatus {
  isConnected: boolean;
}

export function RobotControl() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    isConnected: false,
  });

  const [isConnecting, setIsConnecting] = useState(false);
  const [pidP, setPidP] = useState("");
  const [pidI, setPidI] = useState("");
  const [pidD, setPidD] = useState("");
  const [gcode, setGcode] = useState("");
  const wsRef = useRef<WebSocket | null>(null);

  // Cleanup WebSocket on unmount
  useEffect(() => {
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  const handleConnect = async () => {
    setIsConnecting(true);

    try {
      // Create WebSocket connection to the robot
      const ws = new WebSocket("ws://172.20.10.10/ws");
      wsRef.current = ws;

      ws.onopen = () => {
        console.log("WebSocket connected");
        toast.success("Connected to robot");

        // Send "START_CONNECTION" message when connected
        ws.send("START_CONNECTION");

        setRobotStatus({
          isConnected: true,
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
        });
      };

      ws.onmessage = (event) => {
        console.log("Message from robot:", event.data);
        // Handle messages from the robot here
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
      wsRef.current.close();
      wsRef.current = null;
    }
    setRobotStatus({
      isConnected: false,
    });
  };

  const sendCommand = (command: string) => {
    if (
      wsRef.current &&
      wsRef.current.readyState === WebSocket.OPEN
    ) {
      wsRef.current.send(command);
      console.log("Sent command:", command);
      toast.success(`Command sent: ${command}`);
    } else {
      toast.error("Robot not connected");
    }
  };

  const handleForward = () => {
    sendCommand("FORWARD");
  };

  const handleBackward = () => {
    sendCommand("BACKWARD");
  };

  const handleStop = () => {
    sendCommand("STOP");
  };

  const handlePidPKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") {
      sendCommand(`PID_P:${pidP}`);
    }
  };

  const handlePidIKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") {
      sendCommand(`PID_I:${pidI}`);
    }
  };

  const handlePidDKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") {
      sendCommand(`PID_D:${pidD}`);
    }
  };

  const handleGcodeKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendCommand(`GCODE:${gcode}`);
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

                <div className="space-y-2">
                  <h4 className="font-medium">
                    Robot Connection
                  </h4>
                  <p className="text-sm text-muted-foreground">
                    IP: 172.20.10.10
                  </p>
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

                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Battery className="h-4 w-4" />
                      <span className="text-sm">Battery</span>
                    </div>
                    <span className="text-sm font-medium">
                      {robotStatus.batteryLevel.toFixed(0)}%
                    </span>
                  </div>
                  <Progress value={robotStatus.batteryLevel} />

                  {robotStatus.batteryLevel < 20 && (
                    <Alert>
                      <AlertTriangle className="h-4 w-4" />
                      <AlertDescription>
                        Low battery! Consider charging soon.
                      </AlertDescription>
                    </Alert>
                  )}
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
              <Settings className="h-5 w-5" />
              Robot Controls
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            {!robotStatus.isConnected ? (
              <div className="text-center py-12">
                <Bot className="h-12 w-12 mx-auto text-muted-foreground mb-4" />
                <p className="text-muted-foreground">
                  Connect to a robot to access controls
                </p>
              </div>
            ) : (
              <>
                {/* Movement Controls */}
                <div className="space-y-4">
                  <h4 className="font-medium">
                    Movement Controls
                  </h4>

                  <div className="flex flex-col items-center gap-4">
                    <Button
                      onClick={handleForward}
                      className="w-32 flex items-center justify-center gap-2"
                      size="lg"
                    >
                      <ArrowUp className="h-5 w-5" />
                      Forward
                    </Button>

                    <div className="flex gap-4">
                      <Button
                        onClick={handleBackward}
                        className="w-32 flex items-center justify-center gap-2"
                        size="lg"
                      >
                        <ArrowDown className="h-5 w-5" />
                        Backward
                      </Button>

                      <Button
                        onClick={handleStop}
                        variant="destructive"
                        className="w-32 flex items-center justify-center gap-2"
                        size="lg"
                      >
                        <Square className="h-5 w-5" />
                        Stop
                      </Button>
                    </div>
                  </div>
                </div>

                <Separator />

                {/* PID Control Variables */}
                <div className="space-y-4">
                  <h4 className="font-medium">
                    PID Control Variables
                  </h4>
                  <div className="grid grid-cols-3 gap-4">
                    <div className="space-y-2">
                      <Label htmlFor="pid-p">P (Proportional)</Label>
                      <Input
                        id="pid-p"
                        type="number"
                        step="0.01"
                        placeholder="0.0"
                        value={pidP}
                        onChange={(e) => setPidP(e.target.value)}
                        onKeyDown={handlePidPKeyDown}
                      />
                    </div>
                    <div className="space-y-2">
                      <Label htmlFor="pid-i">I (Integral)</Label>
                      <Input
                        id="pid-i"
                        type="number"
                        step="0.01"
                        placeholder="0.0"
                        value={pidI}
                        onChange={(e) => setPidI(e.target.value)}
                        onKeyDown={handlePidIKeyDown}
                      />
                    </div>
                    <div className="space-y-2">
                      <Label htmlFor="pid-d">D (Derivative)</Label>
                      <Input
                        id="pid-d"
                        type="number"
                        step="0.01"
                        placeholder="0.0"
                        value={pidD}
                        onChange={(e) => setPidD(e.target.value)}
                        onKeyDown={handlePidDKeyDown}
                      />
                    </div>
                  </div>
                  <p className="text-xs text-muted-foreground">
                    Press Enter after typing to send each value to the robot
                  </p>
                </div>

                <Separator />

                {/* GCode Input */}
                <div className="space-y-4">
                  <h4 className="font-medium">
                    GCode Commands
                  </h4>
                  <div className="space-y-2">
                    <Textarea
                      placeholder="Enter GCode commands here..."
                      value={gcode}
                      onChange={(e) => setGcode(e.target.value)}
                      onKeyDown={handleGcodeKeyDown}
                      rows={5}
                      className="font-mono text-sm"
                    />
                    <p className="text-xs text-muted-foreground">
                      Press Enter to send (Shift+Enter for new line)
                    </p>
                  </div>
                </div>
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}