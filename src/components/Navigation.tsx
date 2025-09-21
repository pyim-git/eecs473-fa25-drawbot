import React from 'react';
import { Button } from './ui/button';
import { useAuth } from '../App';
import { Camera, Upload, Bot, LogOut, User } from 'lucide-react';

interface NavigationProps {
  currentPage: 'snapshots' | 'upload' | 'robot';
  onPageChange: (page: 'snapshots' | 'upload' | 'robot') => void;
}

export function Navigation({ currentPage, onPageChange }: NavigationProps) {
  const { user, logout } = useAuth();

  return (
    <nav className="border-b bg-card">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between h-16">
          <div className="flex items-center space-x-1">
            <div className="flex items-center space-x-2 mr-8">
              <Bot className="h-8 w-8 text-primary" />
              <span className="text-xl font-semibold">DrawBot</span>
            </div>
            
            <Button
              variant={currentPage === 'snapshots' ? 'default' : 'ghost'}
              onClick={() => onPageChange('snapshots')}
              className="flex items-center space-x-2"
            >
              <Camera className="h-4 w-4" />
              <span>Snapshots</span>
            </Button>
            
            <Button
              variant={currentPage === 'upload' ? 'default' : 'ghost'}
              onClick={() => onPageChange('upload')}
              className="flex items-center space-x-2"
            >
              <Upload className="h-4 w-4" />
              <span>Upload</span>
            </Button>
            
            <Button
              variant={currentPage === 'robot' ? 'default' : 'ghost'}
              onClick={() => onPageChange('robot')}
              className="flex items-center space-x-2"
            >
              <Bot className="h-4 w-4" />
              <span>Robot Control</span>
            </Button>
          </div>
          
          <div className="flex items-center space-x-4">
            <div className="flex items-center space-x-2 text-muted-foreground">
              <User className="h-4 w-4" />
              <span>{user?.user_metadata?.name || user?.email || 'User'}</span>
            </div>
            <Button variant="ghost" onClick={logout} className="flex items-center space-x-2">
              <LogOut className="h-4 w-4" />
              <span>Logout</span>
            </Button>
          </div>
        </div>
      </div>
    </nav>
  );
}