import React, { useState, useEffect, createContext, useContext } from 'react';
import { Login } from './components/Login';
import { SavedSnapshots } from './components/SavedSnapshots';
import { UploadSnapshot } from './components/UploadSnapshot';
import { RobotControl } from './components/RobotControl';
import { Navigation } from './components/Navigation';
import { createClient } from './utils/supabase/client';

// Authentication Context
interface AuthContextType {
  isAuthenticated: boolean;
  user: any;
  accessToken: string | null;
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  signup: (email: string, password: string, name: string) => Promise<{ success: boolean; error?: string }>;
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | null>(null);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

// Snapshot type
export interface Snapshot {
  id: string;
  name: string;
  imageUrl?: string;
  createdAt: string;
  canvasSize: string;
  storagePath?: string;
  fileSize?: number;
}

export default function App() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState<any>(null);
  const [accessToken, setAccessToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [currentPage, setCurrentPage] = useState<'snapshots' | 'upload' | 'robot'>('snapshots');

  const supabase = createClient();

  // Check for existing session
  useEffect(() => {
    const checkSession = async () => {
      try {
        const { data: { session }, error } = await supabase.auth.getSession();
        if (session?.access_token) {
          setIsAuthenticated(true);
          setUser(session.user);
          setAccessToken(session.access_token);
        }
      } catch (error) {
        console.log('Error checking session:', error);
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();

    // Listen for auth changes
    const { data: { subscription } } = supabase.auth.onAuthStateChange(
      async (event, session) => {
        if (session?.access_token) {
          setIsAuthenticated(true);
          setUser(session.user);
          setAccessToken(session.access_token);
        } else {
          setIsAuthenticated(false);
          setUser(null);
          setAccessToken(null);
        }
      }
    );

    return () => subscription.unsubscribe();
  }, []);

  const login = async (email: string, password: string): Promise<{ success: boolean; error?: string }> => {
    try {
      const { data, error } = await supabase.auth.signInWithPassword({
        email,
        password,
      });

      if (error) {
        return { success: false, error: error.message };
      }

      return { success: true };
    } catch (error) {
      return { success: false, error: 'Login failed. Please try again.' };
    }
  };

  const signup = async (email: string, password: string, name: string): Promise<{ success: boolean; error?: string }> => {
    try {
      const { projectId, publicAnonKey } = await import('./utils/supabase/info');
      
      const response = await fetch(`https://${projectId}.supabase.co/functions/v1/make-server-2eb0fb94/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${publicAnonKey}`,
        },
        body: JSON.stringify({ email, password, name }),
      });

      const result = await response.json();
      
      if (!response.ok) {
        return { success: false, error: result.error || 'Signup failed' };
      }

      // After successful signup, sign in the user
      return await login(email, password);
    } catch (error) {
      return { success: false, error: 'Signup failed. Please try again.' };
    }
  };

  const logout = async (): Promise<void> => {
    try {
      await supabase.auth.signOut();
    } catch (error) {
      console.log('Error during logout:', error);
    } finally {
      // Always reset state even if signOut fails
      setIsAuthenticated(false);
      setUser(null);
      setAccessToken(null);
      setCurrentPage('snapshots');
    }
  };

  const authContextValue: AuthContextType = {
    isAuthenticated,
    user,
    accessToken,
    login,
    signup,
    logout,
  };

  if (isLoading) {
    return (
      <div className="min-h-screen bg-background flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary mx-auto"></div>
          <p className="mt-2 text-muted-foreground">Loading...</p>
        </div>
      </div>
    );
  }

  if (!isAuthenticated) {
    return (
      <AuthContext.Provider value={authContextValue}>
        <div className="min-h-screen bg-background">
          <Login />
        </div>
      </AuthContext.Provider>
    );
  }

  return (
    <AuthContext.Provider value={authContextValue}>
      <div className="min-h-screen bg-background">
        <Navigation currentPage={currentPage} onPageChange={setCurrentPage} />
        <main className="container mx-auto px-4 py-6">
          {currentPage === 'snapshots' && <SavedSnapshots />}
          {currentPage === 'upload' && <UploadSnapshot />}
          {currentPage === 'robot' && <RobotControl />}
        </main>
      </div>
    </AuthContext.Provider>
  );
}