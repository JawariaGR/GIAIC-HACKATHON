import React, { createContext, useState, useContext, ReactNode } from 'react';

interface SelectedTextContextType {
  selectedText: string | null;
  setSelectedText: (text: string | null) => void;
}

const SelectedTextContext = createContext<SelectedTextContextType | undefined>(undefined);

interface SelectedTextProviderProps {
  children: ReactNode;
}

export const SelectedTextProvider: React.FC<SelectedTextProviderProps> = ({ children }) => {
  const [selectedText, setSelectedText] = useState<string | null>(null);

  return (
    <SelectedTextContext.Provider value={{ selectedText, setSelectedText }}>
      {children}
    </SelectedTextContext.Provider>
  );
};

export const useSelectedText = () => {
  const context = useContext(SelectedTextContext);
  if (context === undefined) {
    throw new Error('useSelectedText must be used within a SelectedTextProvider');
  }
  return context;
};
