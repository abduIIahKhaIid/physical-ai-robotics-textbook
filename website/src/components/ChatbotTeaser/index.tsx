import React, { useState } from 'react';
import { Bot, MessageCircle, Sparkles } from 'lucide-react';

export default function ChatbotTeaser() {
  const [chatMessage, setChatMessage] = useState('');

  return (
    <section className="py-16 md:py-24 bg-gradient-to-b from-gray-50 to-gray-100 dark:from-gray-950 dark:to-gray-900">
      <div className="container mx-auto px-6 lg:px-8">
        <div className="grid md:grid-cols-2 gap-12 items-center">

          <div className="text-center md:text-left">
            <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-blue-100/80 dark:bg-blue-900/40 text-blue-800 dark:text-blue-300 mb-4">
              <Sparkles className="w-4 h-4" />
              <span className="text-sm font-medium">AI-Powered Assistance</span>
            </div>
            
            <h2 className="text-3xl md:text-4xl font-bold text-gray-900 dark:text-white mb-4">
              Need Help? Ask Our AI Tutor
            </h2>
            <p className="text-lg text-gray-600 dark:text-gray-400 mb-8">
              Get instant answers to your robotics questions. Our AI tutor understands complex concepts and provides tailored explanations to accelerate your learning.
            </p>
            
            <div className="flex flex-col sm:flex-row gap-4">
              <button className="inline-flex items-center justify-center gap-2 px-6 py-3.5 font-semibold text-white bg-gradient-to-r from-blue-600 to-indigo-600 dark:from-blue-600 dark:to-indigo-700 rounded-xl shadow-lg dark:shadow-indigo-900/40 hover:shadow-xl dark:hover:shadow-indigo-900/60 transition-all duration-300 hover:scale-105">
                <MessageCircle className="w-5 h-5" />
                Chat with RoboTutor
              </button>
              <button className="inline-flex items-center justify-center gap-2 px-6 py-3.5 font-semibold text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-800 rounded-xl shadow hover:shadow-lg dark:hover:shadow-lg transition-all duration-300 border border-gray-200 dark:border-gray-700 dark:hover:bg-gray-700">
                <Bot className="w-5 h-5" />
                Learn More
              </button>
            </div>
          </div>

          <div className="flex justify-center">
            <div className="relative w-full max-w-md p-6 bg-white dark:bg-gray-900 rounded-2xl shadow-lg dark:shadow-lg border border-gray-200/80 dark:border-gray-700/80">
              <div className="absolute -top-4 left-6 bg-gradient-to-r from-blue-600 to-indigo-600 dark:from-blue-600 dark:to-indigo-700 text-white px-4 py-1.5 rounded-full text-sm font-medium shadow-md dark:shadow-indigo-900/50">
                RoboTutor AI
              </div>
              
              <div className="flex items-start gap-4 pt-6">
                <div className="flex-shrink-0 w-12 h-12 bg-gradient-to-br from-blue-500 to-indigo-600 dark:from-blue-600 dark:to-indigo-700 rounded-full flex items-center justify-center">
                  <Bot className="w-6 h-6 text-white" />
                </div>
                <div className="flex-1">
                  <div className="mt-2 p-4 bg-gray-50 dark:bg-gray-800/50 rounded-lg text-gray-700 dark:text-gray-300">
                    <p className="typing-animation overflow-hidden whitespace-nowrap border-r-4 border-r-blue-500 dark:border-r-blue-400">
                      "What's the difference between forward and inverse kinematics?"
                    </p>
                  </div>
                </div>
              </div>
              
              <div className="mt-4 flex items-center p-3 bg-gray-50 dark:bg-gray-800/30 rounded-lg">
                <input
                  type="text"
                  placeholder="Ask about robotics concepts..."
                  value={chatMessage}
                  onChange={(e) => setChatMessage(e.target.value)}
                  className="flex-grow bg-transparent focus:outline-none text-gray-700 dark:text-gray-300 placeholder-gray-500 dark:placeholder-gray-500"
                />
                <button className="ml-2 px-4 py-2 bg-gradient-to-r from-blue-500 to-indigo-500 dark:from-blue-600 dark:to-indigo-700 text-white rounded-lg hover:from-blue-600 hover:to-indigo-600 transition-all">
                  Send
                </button>
              </div>
            </div>
          </div>

        </div>
      </div>
      
      <style>{`
        .typing-animation {
          width: 0;
          animation: typing 2.5s steps(30, end) forwards, blink 0.75s step-end infinite;
        }
        
        @keyframes typing {
          from { width: 0; }
          to { width: 100%; }
        }
        
        @keyframes blink {
          from, to { border-color: transparent; }
          50% { border-color: inherit; }
        }
      `}</style>
    </section>
  );
}