import React, { useState, useEffect, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { Button } from './ui/button';
import { Badge } from './ui/badge';
import { Progress } from './ui/progress';
import { Alert, AlertDescription } from './ui/alert';
import { Separator } from './ui/separator';
import { 
  Bot, 
  Wifi, 
  WifiOff, 
  Battery, 
  Play, 
  Pause, 
  Square, 
  RotateCcw,
  Settings,
  AlertTriangle,
  CheckCircle,
  Zap,
  ArrowUp,
  ArrowDown
} from 'lucide-react';
import { toast } from 'sonner@2.0.3';

interface RobotStatus {
  isConnected: boolean;
  batteryLevel: number;
  isDrawing: boolean;
  currentTask: string | null;
  position: { x: number; y: number };
  drawingProgress: number;
}

export function RobotControl() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    isConnected: false,
    batteryLevel: 0,
    isDrawing: false,
    currentTask: null,
    position: { x: 0, y: 0 },
    drawingProgress: 0
  });

  const [isConnecting, setIsConnecting] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);

  // Cleanup WebSocket on unmount
  useEffect(() => {
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  // Simulate robot status updates
  useEffect(() => {
    if (robotStatus.isConnected) {
      const interval = setInterval(() => {
        setRobotStatus(prev => ({
          ...prev,
          batteryLevel: Math.max(0, prev.batteryLevel - 0.1),
          position: {
            x: prev.position.x + (Math.random() - 0.5) * 2,
            y: prev.position.y + (Math.random() - 0.5) * 2
          },
          drawingProgress: prev.isDrawing 
            ? Math.min(100, prev.drawingProgress + 1)
            : prev.drawingProgress
        }));
      }, 1000);

      return () => clearInterval(interval);
    }
  }, [robotStatus.isConnected, robotStatus.isDrawing]);

  const handleConnect = async () => {
    setIsConnecting(true);
    
    try {
      // Create WebSocket connection to the robot
      const ws = new WebSocket('ws://172.20.10.10/ws');
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('WebSocket connected');
        toast.success('Connected to robot');
        
        // Send "START_CONNECTION" message when connected
        ws.send('START_CONNECTION');
        
        setRobotStatus({
          isConnected: true,
          batteryLevel: 85,
          isDrawing: false,
          currentTask: null,
          position: { x: 0, y: 0 },
          drawingProgress: 0
        });
        
        setIsConnecting(false);
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        toast.error('Failed to connect to robot');
        setIsConnecting(false);
      };

      ws.onclose = () => {
        console.log('WebSocket disconnected');
        toast.info('Disconnected from robot');
        setRobotStatus(prev => ({
          ...prev,
          isConnected: false,
          isDrawing: false,
          currentTask: null,
          drawingProgress: 0
        }));
      };

      ws.onmessage = (event) => {
        console.log('Message from robot:', event.data);
        // Handle messages from the robot here
        toast.info(`Robot: ${event.data}`);
      };

    } catch (error) {
      console.error('Connection error:', error);
      toast.error('Failed to connect to robot');
      setIsConnecting(false);
    }
  };

  const handleDisconnect = () => {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setRobotStatus(prev => ({
      ...prev,
      isConnected: false,
      isDrawing: false,
      currentTask: null,
      drawingProgress: 0
    }));
  };

  const handleStartDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: true,
      currentTask: 'Drawing geometric pattern',
      drawingProgress: 0
    }));
  };

  const handlePauseDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: false,
      currentTask: 'Paused'
    }));
  };

  const handleStopDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: false,
      currentTask: null,
      drawingProgress: 0
    }));
  };

  const handleHomePosition = () => {
    setRobotStatus(prev => ({
      ...prev,
      position: { x: 0, y: 0 },
      currentTask: 'Returning to home position'
    }));
    
    setTimeout(() => {
      setRobotStatus(prev => ({
        ...prev,
        currentTask: null
      }));
    }, 2000);
  };

  const sendCommand = (command: string) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(command);
      console.log('Sent command:', command);
      toast.success(`Command sent: ${command}`);
    } else {
      toast.error('Robot not connected');
    }
  };

  const handleForward = () => {
    sendCommand('FORWARD');
  };

  const handleBackward = () => {
    sendCommand('BACKWARD');
  };

  const handleStop = () => {
    sendCommand('STOP');
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
                  <p className="text-sm text-muted-foreground">No robot connected</p>
                </div>
                
                <div className="space-y-2">
                  <h4 className="font-medium">Robot Connection</h4>
                  <p className="text-sm text-muted-foreground">IP: 172.20.10.10</p>
                  <Button
                    className="w-full"
                    onClick={handleConnect}
                    disabled={isConnecting}
                  >
                    {isConnecting ? 'Connecting...' : 'Connect'}
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
                  <Badge variant="outline" className="text-green-600 border-green-600">
                    Connected
                  </Badge>
                </div>

                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Battery className="h-4 w-4" />
                      <span className="text-sm">Battery</span>
                    </div>
                    <span className="text-sm font-medium">{robotStatus.batteryLevel.toFixed(0)}%</span>
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

                <Button variant="outline" onClick={handleDisconnect} className="w-full">
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
                <p className="text-muted-foreground">Connect to a robot to access controls</p>
              </div>
            ) : (
              <>
                {/* Current Status */}
                <div className="space-y-3">
                  <h4 className="font-medium">Current Status</h4>
                  <div className="grid grid-cols-2 gap-4">
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Status</p>
                      <div className="flex items-center gap-2">
                        {robotStatus.isDrawing ? (
                          <>
                            <Zap className="h-4 w-4 text-green-500" />
                            <span className="text-green-600">Drawing</span>
                          </>
                        ) : (
                          <>
                            <div className="h-4 w-4 rounded-full bg-muted-foreground"></div>
                            <span>Idle</span>
                          </>
                        )}
                      </div>
                    </div>
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Position</p>
                      <p className="font-mono text-sm">
                        X: {robotStatus.position.x.toFixed(1)} Y: {robotStatus.position.y.toFixed(1)}
                      </p>
                    </div>
                  </div>
                  
                  {robotStatus.currentTask && (
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Current Task</p>
                      <p className="text-sm">{robotStatus.currentTask}</p>
                    </div>
                  )}
                </div>

                <Separator />

                {/* Movement Controls */}
                <div className="space-y-4">
                  <h4 className="font-medium">Movement Controls</h4>
                  
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
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}